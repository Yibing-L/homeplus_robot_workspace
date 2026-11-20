"""Pure decision logic for the executor's automatic fault recovery.

No ROS imports: the executor (gesture_motion_executor.py) does the waiting,
planning and publishing; this module only decides what to do next, so the
rules can be unit-tested offline (tests/test_fault_recovery.py).

The firmware's limits (3 deg arm tracking, SETTLE_TIMEOUT, STOP) are not
changed by anything here. A recovered step is always replanned from the
measured joint state; the old trajectory is never resent.
"""
from typing import Dict, Iterable, Optional, Sequence, Tuple

# Firmware faults after which the arm holds where it stopped
# (stopAtCurrentState) and a slower replan of the same step may succeed.
RETRYABLE_FAULTS = ('TRACKING_ERROR', 'SETTLE_TIMEOUT')
# Floor for the halved velocity/acceleration scaling on a retry.
MIN_FAULT_SCALING = 0.1
OCTOMAP_BODY = '<octomap>'
TABLE_BODY = 'detected_table'
DETECTION_TIMEOUT_ACTIONS = ('abort', 'back_up')

# Defaults when neither the step nor its sequence sets the key.
RECOVERY_DEFAULTS = {
    'fault_retries': 2,
    'fault_settle_timeout_sec': 10.0,
    'start_state_recovery': True,
    'on_detection_timeout': 'abort',
}


class MotionFault(RuntimeError):
    """A motion failed on the hardware (Arduino ERROR or no verified endpoint)
    inside a multi-phase step; carries the bridge's error line."""

    def __init__(self, message: str, error: Optional[str]):
        super().__init__(message if not error else f'{message}: {error}')
        self.error = error


def fault_code(error: Optional[str]) -> Optional[str]:
    """'ERROR <id> TRACKING_ERROR dx_cm=...' -> 'TRACKING_ERROR'; 'ERROR 7 SERIAL x' -> 'SERIAL'.

    Host-side failures (no DONE in time, DONE without a settled endpoint) have
    no firmware code and return None.
    """
    if not error:
        return None
    fields = str(error).split()
    if len(fields) >= 3 and fields[0].upper() == 'ERROR' and fields[1].isdigit():
        return fields[2]
    return None


def recovery_setting(step: Dict, sequence_cfg: Optional[Dict], key: str):
    """Step key, else the sequence's, else the default."""
    if key in step:
        return step[key]
    if sequence_cfg and key in sequence_cfg:
        return sequence_cfg[key]
    return RECOVERY_DEFAULTS[key]


def fault_retry_decision(code: Optional[str], retries_done: int, max_retries: int,
                         stop_requested: bool = False) -> Tuple[bool, str]:
    """(retry?, reason when not). STOP, SERIAL and every other code abort as before."""
    if stop_requested:
        return False, 'operator STOP: not retrying'
    if code not in RETRYABLE_FAULTS:
        return False, f'{code or "no firmware fault code"}: not retried'
    if retries_done >= max_retries:
        return False, f'{code} after {retries_done} recovery attempt(s)'
    return True, ''


def recovery_scaling(base: float, attempt: int) -> float:
    """Scaling for the attempt-th retry: halved each time, floor 0.1, never above base."""
    base = float(base)
    return min(base, max(MIN_FAULT_SCALING, base * 0.5 ** int(attempt)))


def scaled_step(step: Dict, attempt: int, default_scaling: float) -> Tuple[Dict, int]:
    """Copy of the step with velocity/acceleration scaling reduced for a retry.

    Returns (step copy, velocity as a percentage of the configured value).
    '_fault_scale' carries the same factor to phases planned inside the step
    (standoff base drive) that do not read the step's scaling keys.
    """
    out = dict(step)
    velocity = float(step.get('velocity_scaling', default_scaling))
    acceleration = float(step.get('acceleration_scaling', default_scaling))
    out['velocity_scaling'] = recovery_scaling(velocity, attempt)
    out['acceleration_scaling'] = recovery_scaling(acceleration, attempt)
    out['_fault_scale'] = recovery_scaling(1.0, attempt)
    percent = int(round(100.0 * out['velocity_scaling'] / velocity)) if velocity > 0 else 100
    return out, percent


def start_state_recoverable(contact_pairs: Iterable[Sequence[str]], constraints_ok: bool) -> bool:
    """True when the start state is invalid only because robot/attached bodies touch
    map voxels: every contact involves '<octomap>', none the table box, and no
    path constraint is violated. Self-collision or table contact is never cleared."""
    pairs = [tuple(pair) for pair in contact_pairs]
    if not constraints_ok or not pairs:
        return False
    return all(OCTOMAP_BODY in pair and TABLE_BODY not in pair for pair in pairs)


def recovery_config_errors(name: str, cfg: Dict, is_sequence: bool) -> list:
    """Validation for the recovery keys (used by _command_config_errors)."""
    errors = []
    if 'fault_retries' in cfg:
        value = cfg['fault_retries']
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 5:
            errors.append(f'{name}: fault_retries must be an integer in [0, 5]')
    if 'fault_settle_timeout_sec' in cfg:
        try:
            ok = 0.5 <= float(cfg['fault_settle_timeout_sec']) <= 60.0
        except (TypeError, ValueError):
            ok = False
        if not ok:
            errors.append(f'{name}: fault_settle_timeout_sec must be in [0.5, 60]')
    if 'start_state_recovery' in cfg and not isinstance(cfg['start_state_recovery'], bool):
        errors.append(f'{name}: start_state_recovery must be true or false')
    if 'on_detection_timeout' in cfg:
        if not is_sequence:
            errors.append(f'{name}: on_detection_timeout applies to sequences only')
        elif cfg['on_detection_timeout'] not in DETECTION_TIMEOUT_ACTIONS:
            errors.append(f'{name}: on_detection_timeout must be one of {list(DETECTION_TIMEOUT_ACTIONS)}')
    return errors
