"""Pure decision logic for handing the held cup to a person (hand_cup_to_person).

No ROS imports: the executor (gesture_motion_executor.py) plans, moves and
waits; this module only computes the geometry and decides, so the rules can be
unit-tested offline (tests/test_handoff_logic.py).

Step types it serves:
  scan_rotate   turn the base in place in steps until a stable detection
  base_approach drive the base toward a detected person to a standoff
  handoff_wait  hold still with the cup at the hand and decide whether to open
                the gripper. With require_release_approval (the default) only
                the operator's approval opens it; contact at the guarded lower
                and a pull on the arm are measured and reported as evidence.
                Without it, evidence alone may open it. Otherwise (timeout,
                Reject, STOP) the cup stays in the gripper.
"""
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

SCAN_DEFAULTS = {'step_deg': 30.0, 'max_deg': 360.0, 'direction': 'left'}
APPROACH_DEFAULTS = {'standoff_m': 0.65, 'max_drive_m': 2.5, 'min_drive_m': 0.05}
HANDOFF_DEFAULTS = {
    'timeout_sec': 30.0,
    'pull_threshold_deg': 1.5,
    'pull_min_duration_sec': 0.3,
    'pull_joints': ['joint_shoulder', 'joint_elbow'],
    'release_on_contact': True,
    'operator_confirm': True,
    # Opening the gripper always asks the operator (operator, 2026-11-15).
    'require_release_approval': True,
    'settle_timeout_sec': 10.0,
    'baseline_sec': 0.5,
}
# Joints with encoders (closed loop). The wrist, hand and gripper are open-loop
# servos whose "measured" value is an estimate: a pull cannot show there.
ENCODER_JOINTS = ('joint_shoulder', 'joint_elbow')

SCAN_KEYS = {'step_deg', 'max_deg', 'direction', 'max_target_range_m', 'min_target_range_m'}
APPROACH_KEYS = {'standoff_m', 'max_drive_m', 'min_drive_m'}
HANDOFF_KEYS = set(HANDOFF_DEFAULTS) | {'contact_from_step'}


def setting(step: Dict, defaults: Dict, key: str):
    return step[key] if key in step else defaults[key]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_turns(step_deg: float, max_deg: float, direction: str = 'left') -> List[float]:
    """Relative base turns (rad) after the first look at the current heading.

    30 deg steps up to 360 deg: 11 turns (12 headings; the 12th turn would
    come back to the start, which was already looked at)."""
    step = abs(float(step_deg))
    total = abs(float(max_deg))
    if step <= 0.0:
        raise ValueError('step_deg must be positive')
    count = int(math.floor(total / step + 1e-9))
    if total >= 360.0 - 1e-6:
        count -= 1  # the last turn would return to the start heading
    sign = 1.0 if direction == 'left' else -1.0
    return [sign * math.radians(step)] * max(0, count)


def target_range(base_xy: Sequence[float], target_xy: Sequence[float]) -> float:
    return math.hypot(target_xy[0] - base_xy[0], target_xy[1] - base_xy[1])


def approach_goal(base: Sequence[float], target_xy: Sequence[float], standoff_m: float,
                  max_drive_m: float, min_drive_m: float = 0.0,
                  tcp_forward_m: float = 0.0) -> Tuple[Optional[Dict[str, float]], str]:
    """Base goal facing the target with the held cup `standoff_m` short of it.

    base = (x, y, theta) measured; target_xy = the person's measured front
    (world). The base drives along the straight line from base_link to the
    person and turns to face them (the arm plane points along base +X); the
    gripper, kept as it is, sits `tcp_forward_m` ahead of base_link, so the
    base stops `standoff_m + tcp_forward_m` from the person. It never drives
    toward a person who is already closer: then it only turns to face them.

    Returns (goal or None, explanation)."""
    bx, by, theta = (float(v) for v in base)
    tx, ty = (float(v) for v in target_xy)
    dist = math.hypot(tx - bx, ty - by)
    if dist < 1e-6:
        return None, 'person is at the base position'
    heading = math.atan2(ty - by, tx - bx)
    stop_at = float(standoff_m) + max(0.0, float(tcp_forward_m))
    drive = dist - stop_at
    if drive > float(max_drive_m):
        return None, (f'person is {dist:.2f} m away: driving {drive:.2f} m exceeds max_drive_m '
                      f'{float(max_drive_m):.2f}')
    if drive < float(min_drive_m):
        drive = 0.0
    goal = {'base_x': bx + drive * math.cos(heading),
            'base_y': by + drive * math.sin(heading),
            # continuous with the measured heading (no 2*pi jump in the plan)
            'base_theta': theta + wrap(heading - theta)}
    return goal, (f'person {dist:.2f} m from base_link at bearing {math.degrees(wrap(heading - theta)):+.0f} deg; '
                  f'turn to face them and drive {drive:.2f} m: base_link stops {dist - drive:.2f} m and the '
                  f'held cup ~{dist - drive - max(0.0, float(tcp_forward_m)):.2f} m from them')


class PullDetector:
    """Evidence that someone is pulling on (or taking the weight of) the held cup.

    The arm holds its position closed-loop at idle (no firmware fault at idle),
    so a pull shows as the measured encoder joints moving away from where they
    settled. Baseline = per-joint median of the first `baseline_sec` of samples;
    a pull = any listed joint more than `threshold` rad from it for at least
    `min_duration` s without interruption."""

    def __init__(self, joints: Iterable[str], threshold_rad: float, min_duration_sec: float,
                 baseline_sec: float = 0.5):
        self.joints = list(joints)
        self.threshold = float(threshold_rad)
        self.min_duration = float(min_duration_sec)
        self.baseline_sec = float(baseline_sec)
        self._start: Optional[float] = None
        self._baseline_samples: List[Dict[str, float]] = []
        self.baseline: Optional[Dict[str, float]] = None
        self._over_since: Optional[float] = None
        self.max_deviation = 0.0
        self.last_deviation: Dict[str, float] = {}

    def update(self, t: float, positions: Dict[str, float]) -> bool:
        """Feed one measured sample (time s, joint -> rad). True once a pull is seen."""
        if any(j not in positions or not math.isfinite(positions[j]) for j in self.joints):
            return False
        if self.baseline is None:
            if self._start is None:
                self._start = t
            self._baseline_samples.append({j: positions[j] for j in self.joints})
            if t - self._start >= self.baseline_sec and len(self._baseline_samples) >= 3:
                self.baseline = {j: _median([s[j] for s in self._baseline_samples]) for j in self.joints}
            return False
        self.last_deviation = {j: positions[j] - self.baseline[j] for j in self.joints}
        worst = max(abs(v) for v in self.last_deviation.values())
        self.max_deviation = max(self.max_deviation, worst)
        if worst > self.threshold:
            if self._over_since is None:
                self._over_since = t
            return t - self._over_since >= self.min_duration
        self._over_since = None
        return False

    def describe(self) -> str:
        parts = ', '.join(f'{j} {math.degrees(v):+.2f} deg' for j, v in self.last_deviation.items())
        return f'{parts} (limit {math.degrees(self.threshold):.2f} deg)'


def _median(values: List[float]) -> float:
    values = sorted(values)
    m = len(values) // 2
    return values[m] if len(values) % 2 else 0.5 * (values[m - 1] + values[m])


def evidence_text(contact: bool, pull: bool) -> str:
    seen = [what for what, yes in (('contact at the guarded lower', contact), ('a pull on the cup', pull)) if yes]
    return ' and '.join(seen) if seen else 'no contact or pull measured'


def release_decision(contact: bool, pull: bool, operator: Optional[bool],
                     release_on_contact: bool,
                     require_approval: bool = False) -> Tuple[Optional[bool], str]:
    """(release?, why). None = keep waiting. operator: True confirm, False refuse,
    None nothing yet. A refusal ends the wait holding the cup. With
    require_approval only the operator's confirmation opens the gripper;
    contact and pull are reported, never acted on alone."""
    if operator is False:
        return False, 'operator refused the release'
    if require_approval:
        if operator is True:
            return True, f'operator approved opening the gripper ({evidence_text(contact, pull)})'
        return None, ''
    if pull:
        return True, 'pull detected'
    if contact and release_on_contact:
        return True, 'contact with the hand at the guarded lower'
    if operator is True:
        return True, 'operator confirmed the release'
    return None, ''


def handoff_config_errors(name: str, cfg: Dict, step_type: str, earlier_steps: Sequence[str]) -> List[str]:
    """Validation for the hand-off step keys (used by _command_config_errors)."""
    errors: List[str] = []
    present = set(cfg)
    misplaced = []
    if step_type != 'scan_rotate':
        misplaced += sorted(present & (SCAN_KEYS - {'max_target_range_m', 'min_target_range_m'}))
    if step_type not in ('scan_rotate', 'detect'):
        misplaced += sorted(present & {'max_target_range_m', 'min_target_range_m'})
    if step_type != 'base_approach':
        misplaced += sorted(present & APPROACH_KEYS)
    if step_type != 'handoff_wait':
        misplaced += sorted(present & HANDOFF_KEYS)
    if misplaced:
        errors.append(f'{name}: keys {misplaced} do not apply to a {step_type} step')

    def positive(key):
        if key in cfg:
            try:
                ok = math.isfinite(float(cfg[key])) and float(cfg[key]) > 0.0
            except (TypeError, ValueError):
                ok = False
            if not ok:
                errors.append(f'{name}: {key} must be a positive number')

    if step_type == 'scan_rotate':
        before = len(errors)
        for key in ('step_deg', 'max_deg', 'max_target_range_m', 'min_target_range_m'):
            positive(key)
        if len(errors) == before:
            if float(cfg.get('step_deg', SCAN_DEFAULTS['step_deg'])) > 90.0:
                errors.append(f'{name}: step_deg above 90 leaves gaps between camera views')
            if float(cfg.get('max_deg', SCAN_DEFAULTS['max_deg'])) > 360.0:
                errors.append(f'{name}: max_deg must be at most 360')
            if ('min_target_range_m' in cfg and 'max_target_range_m' in cfg
                    and float(cfg['min_target_range_m']) >= float(cfg['max_target_range_m'])):
                errors.append(f'{name}: min_target_range_m must be below max_target_range_m')
        if cfg.get('direction', 'left') not in ('left', 'right'):
            errors.append(f'{name}: direction must be left or right')
    if step_type == 'base_approach':
        for key in ('standoff_m', 'max_drive_m'):
            positive(key)
        if 'min_drive_m' in cfg and not float(cfg['min_drive_m']) >= 0.0:
            errors.append(f'{name}: min_drive_m must be nonnegative')
        if 'standoff_m' in cfg and float(cfg['standoff_m']) < 0.3:
            # held cup to the person's measured front, horizontally
            errors.append(f'{name}: standoff_m below 0.3 m is not allowed near a person')
        for key in ('velocity_scaling', 'acceleration_scaling'):
            if key in cfg and float(cfg[key]) > 0.3:
                errors.append(f'{name}: {key} above 0.3 is not allowed for driving toward a person')
    if step_type == 'handoff_wait':
        for key in ('timeout_sec', 'pull_threshold_deg', 'pull_min_duration_sec', 'settle_timeout_sec',
                    'baseline_sec'):
            positive(key)
        joints = cfg.get('pull_joints', HANDOFF_DEFAULTS['pull_joints'])
        if not isinstance(joints, list) or not joints or set(joints) - set(ENCODER_JOINTS):
            errors.append(f'{name}: pull_joints must be a non-empty subset of {list(ENCODER_JOINTS)} '
                          '(the other joints have no encoder)')
        for key in ('release_on_contact', 'operator_confirm', 'require_release_approval'):
            if key in cfg and not isinstance(cfg[key], bool):
                errors.append(f'{name}: {key} must be true or false')
        if cfg.get('require_release_approval', True) and not cfg.get('operator_confirm', True):
            errors.append(f'{name}: require_release_approval needs operator_confirm (the approval prompt)')
        source = cfg.get('contact_from_step')
        if source is not None and source not in earlier_steps:
            errors.append(f'{name}: contact_from_step must name an earlier step')
        if cfg.get('release_on_contact', True) and source is None:
            errors.append(f'{name}: release_on_contact needs contact_from_step (the guarded step)')
    return errors
