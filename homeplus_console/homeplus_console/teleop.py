"""Live teleop controller (pure logic; teleop_service.py does the I/O).

Every tick (25 Hz) it turns the operator's intent into ONE absolute joint setpoint for the bridge's
jog stream (/arduino/jog), in SI units and the bridge's joint order. Nothing here talks to ROS.

Per tick, in order:
  1. base: key axes → body velocity (forward, strafe, turn), acceleration-limited, integrated into
     the commanded odometry pose (x, y, theta). The base is holonomic (mecanum-style X drive;
     the firmware runs x/y/theta PIDs on odometry), so A/D strafe and Q/E turn.
  2. arm: mouse drag adds a pending TCP offset (reach, height); the wheel and R/F add pending pitch.
     The TCP moves toward it at a capped, acceleration-limited Cartesian speed; IK (teleop_kin)
     turns that into shoulder/elbow/hand. Z/X roll the wrist, O/C/G move the gripper.
  3. joint limits (URDF/joint_limits.yaml, minus a margin), joint speed and acceleration caps.
  4. tracking governor: the firmware trips TRACKING_ERROR at 3 deg (shoulder/elbow), 2 deg (yaw),
     6 cm (x/y) between its interpolated target and the encoders. The firmware target lags our
     command by the stream lookahead, so we estimate it from our own command history and stop
     advancing an axis while its estimated lag exceeds ~60 % of the firmware limit.
  5. collision probe: the state the robot would reach `probe_sec` ahead at the current velocity is
     checked (MoveIt /check_state_validity: self-collision, world objects, OctoMap). Invalid →
     hold the command (velocity zero) and report the contacts. If the robot is already in contact,
     moves that do not deepen it are allowed (escape).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

from . import teleop_kin as kin

# The bridge's JOINT_NAMES order; SI units (m, rad).
ORDER = ("base_x", "base_y", "base_theta", "joint_grip_L", "joint_hand", "joint_wrist", "joint_elbow", "joint_shoulder")
ARM = ("joint_shoulder", "joint_elbow", "joint_hand", "joint_wrist", "joint_grip_L")
D = math.radians

DEFAULTS: Dict[str, Any] = {
    "rate_hz": 25.0,
    "deadman_sec": 0.30,          # no input frame for this long → stop
    "idle_end_sec": 0.5,          # nothing commanded for this long → finish the stream (robot holds)
    "base_lin_mps": [0.05, 0.12],  # [normal, Shift]
    "base_lin_acc": 0.20,
    "base_rot_dps": [10.0, 20.0],
    "base_rot_acc_dps2": 40.0,
    "tcp_mps": [0.03, 0.06],
    "tcp_acc": 0.15,
    "mouse_m_per_px": [0.00015, 0.0004],
    "pending_max_m": 0.03,        # mouse motion beyond this, not yet executed, is dropped
    "wheel_deg_per_notch": 3.0,
    "pending_max_deg": 10.0,
    "pitch_dps": [15.0, 25.0],
    "roll_dps": [30.0, 60.0],
    "grip_dps": [40.0, 80.0],
    "grip_open_deg": 0.0,
    "grip_close_deg": 75.0,       # G toggles between these; O/C ramp anywhere in [0, grip_max_deg]
    "grip_max_deg": 80.0,
    "joint_max_dps": {"joint_shoulder": 8.0, "joint_elbow": 10.0, "joint_hand": 30.0, "joint_wrist": 60.0, "joint_grip_L": 90.0},
    "joint_max_acc_dps2": {"joint_shoulder": 20.0, "joint_elbow": 25.0, "joint_hand": 120.0, "joint_wrist": 200.0, "joint_grip_L": 400.0},
    "limit_margin_deg": 2.0,
    # joint_limits.yaml (MoveIt) — the firmware rejects elbow > 140 deg
    "limits_deg": {"joint_shoulder": [-30.0, 150.0], "joint_elbow": [-35.0, 140.0], "joint_hand": [-70.0, 80.0],
                   "joint_wrist": [-90.0, 90.0], "joint_grip_L": [0.0, 90.0]},
    "lookahead_sec": 0.35,        # bridge lookahead (0.30) + packet latency: how far the firmware target lags us
    "governor": {"base_x": 0.035, "base_y": 0.035, "base_theta_deg": 1.2, "joint_elbow_deg": 1.8, "joint_shoulder_deg": 1.8},
    "lost": {"base_xy": 0.10, "base_theta_deg": 6.0, "arm_deg": 6.0},
    "probe_sec": 0.5,
    # the probe also reaches at least this far ahead, so the robot stops short of contact, not at it
    "probe_min_base_m": 0.03, "probe_min_base_deg": 3.0, "probe_min_tcp_m": 0.015,
    "collision_mode": "full",     # full | no_octomap | self_only
    # closing the gripper on an object is the point: finger contacts don't block gripper-only motion
    "finger_links": ["FingerL_new_1", "FingerR_new_1"],
}


def merged(cfg: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULTS.items()}
    for k, v in (cfg or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k].update(v)
        else:
            out[k] = v
    return out


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class Intent:
    """What the operator is holding right now (from the browser), each axis in [-1, 1]."""
    fwd: float = 0.0
    strafe: float = 0.0     # + = left
    turn: float = 0.0       # + = counter-clockwise (left)
    pitch: float = 0.0      # + = tool tip up
    roll: float = 0.0
    grip: float = 0.0       # + = close
    fast: bool = False
    drag: bool = False      # a mouse button is held on the camera
    keys: Tuple[str, ...] = ()

    @staticmethod
    def parse(m: Dict[str, Any]) -> "Intent":
        f = lambda k: clamp(float(m.get(k, 0.0) or 0.0), -1.0, 1.0)  # noqa: E731
        keys = tuple(str(k)[:12] for k in (m.get("keys") or [])[:16])
        return Intent(f("fwd"), f("strafe"), f("turn"), f("pitch"), f("roll"), f("grip"),
                      bool(m.get("fast")), bool(m.get("drag")), keys)

    def any(self) -> bool:
        return any(abs(v) > 1e-6 for v in (self.fwd, self.strafe, self.turn, self.pitch, self.roll, self.grip)) or self.drag


@dataclass
class Check:
    valid: bool
    contacts: List[Tuple[str, str, float]] = field(default_factory=list)
    error: Optional[str] = None
    ms: float = 0.0


Checker = Callable[[Dict[str, float]], Check]


@dataclass
class StepOut:
    q: Dict[str, float]
    moving: bool
    blocked: Optional[str] = None
    contacts: List[Tuple[str, str, float]] = field(default_factory=list)
    escape: bool = False
    governed: List[str] = field(default_factory=list)
    lost: Optional[str] = None
    limited: List[str] = field(default_factory=list)
    ik_short: bool = False
    check_ms: Optional[float] = None


def filter_contacts(contacts: Sequence[Tuple[str, str, float]], mode: str,
                    robot_links: Optional[set] = None) -> List[Tuple[str, str, float]]:
    """Contacts that block in this mode. full: all; no_octomap: all but OctoMap voxels;
    self_only: robot link against robot link only (world objects, voxels and held objects ignored)."""
    if mode == "no_octomap":
        return [c for c in contacts if "<octomap>" not in (c[0], c[1])]
    if mode == "self_only":
        links = robot_links or set()
        return [c for c in contacts if c[0] in links and c[1] in links]
    return list(contacts)


class Controller:
    def __init__(self, chain: kin.ArmChain, cfg: Optional[Dict[str, Any]] = None):
        self.chain = chain
        self.cfg = merged(cfg)
        c = self.cfg
        m = D(c["limit_margin_deg"])
        self.bounds = {n: (D(lo) + m, D(hi) - m) for n, (lo, hi) in c["limits_deg"].items()}
        self.bounds["joint_grip_L"] = (0.0, D(min(c["grip_max_deg"], c["limits_deg"]["joint_grip_L"][1])))
        self.q: Optional[Dict[str, float]] = None
        self.reset_motion()

    # ------------------------------------------------------------------ state
    def reset_motion(self) -> None:
        self.v_body = [0.0, 0.0, 0.0]           # vx, vy (m/s, body), wz (rad/s)
        self.v_joint = {n: 0.0 for n in ARM}
        self.tcp_speed = 0.0
        self.pend = [0.0, 0.0, 0.0]             # reach (m), height (m), pitch (rad) not yet executed
        self.grip_goal: Optional[float] = None  # G toggles a goal; O/C cancel it
        self.history: Deque[Tuple[float, Dict[str, float]]] = deque(maxlen=200)
        self.last: Dict[str, Any] = {}

    def anchor(self, measured: Dict[str, float], now: float) -> Dict[str, float]:
        """Start a session from the measured pose (the bridge rejects a first point that isn't)."""
        # Exactly the measured pose: a joint already outside the teleop bounds may only move inward.
        self.q = {n: float(measured[n]) for n in ORDER}
        self.reset_motion()
        self.history.append((now, dict(self.q)))
        return dict(self.q)

    def add_mouse(self, dx_px: float, dy_px: float, wheel: float, fast: bool) -> None:
        """Mouse right = reach out, mouse up = up (screen y grows down), wheel up = tip up."""
        k = self.cfg["mouse_m_per_px"][1 if fast else 0]
        lim = self.cfg["pending_max_m"]
        self.pend[0] = clamp(self.pend[0] + dx_px * k, -lim, lim)
        self.pend[1] = clamp(self.pend[1] - dy_px * k, -lim, lim)
        plim = D(self.cfg["pending_max_deg"])
        self.pend[2] = clamp(self.pend[2] + D(self.cfg["wheel_deg_per_notch"]) * wheel, -plim, plim)

    def toggle_grip(self) -> None:
        if self.q is None:
            return
        mid = D((self.cfg["grip_open_deg"] + self.cfg["grip_close_deg"]) / 2)
        self.grip_goal = D(self.cfg["grip_open_deg"] if self.q["joint_grip_L"] > mid else self.cfg["grip_close_deg"])

    def clear_pending(self) -> None:
        self.pend = [0.0, 0.0, 0.0]

    def busy(self) -> bool:
        """Still moving or with motion queued (the session must not end yet)."""
        return (any(abs(v) > 1e-4 for v in self.v_body) or any(abs(v) > 1e-4 for v in self.v_joint.values())
                or self.tcp_speed > 1e-4 or any(abs(p) > 1e-4 for p in self.pend) or self.grip_goal is not None)

    def tcp(self, q: Optional[Dict[str, float]] = None) -> Optional[Tuple[float, float, float]]:
        q = q or self.q
        if q is None:
            return None
        t = kin.task(self.chain.fk(q))
        return float(t[0]), float(t[1]), float(t[2])

    def fw_target_estimate(self, now: float) -> Optional[Dict[str, float]]:
        """Our command as it was `lookahead_sec` ago ≈ what the firmware is interpolating now."""
        t = now - self.cfg["lookahead_sec"]
        older = [q for (ts, q) in self.history if ts <= t]
        return older[-1] if older else (self.history[0][1] if self.history else None)

    # ------------------------------------------------------------------ tick
    def step(self, now: float, dt: float, it: Intent, measured: Optional[Dict[str, float]], check: Optional[Checker]) -> StepOut:
        assert self.q is not None, "anchor() first"
        c = self.cfg
        dt = clamp(dt, 1e-3, 0.1)
        fast = 1 if it.fast else 0
        q0 = dict(self.q)
        q = dict(q0)
        out = StepOut(q=q0, moving=False)

        # -- 1. base: accel-limited body velocity, integrated in odometry
        vmax, wmax = c["base_lin_mps"][fast], D(c["base_rot_dps"][fast])
        want = [it.fwd * vmax, it.strafe * vmax, it.turn * wmax]
        acc = [c["base_lin_acc"] * dt, c["base_lin_acc"] * dt, D(c["base_rot_acc_dps2"]) * dt]
        self.v_body = [v + clamp(w - v, -a, a) for v, w, a in zip(self.v_body, want, acc)]
        vx, vy, wz = self.v_body
        th = q["base_theta"] + 0.5 * wz * dt
        q["base_x"] += (math.cos(th) * vx - math.sin(th) * vy) * dt
        q["base_y"] += (math.sin(th) * vx + math.cos(th) * vy) * dt
        q["base_theta"] += wz * dt

        # -- 2. arm: pending Cartesian offset + pitch keys → IK
        if not it.drag:                 # button released: stop where we are (wheel pitch still completes)
            self.pend[0] = self.pend[1] = 0.0
        self.pend[2] += D(c["pitch_dps"][fast]) * it.pitch * dt
        dist = math.hypot(self.pend[0], self.pend[1])
        vcap = min(c["tcp_mps"][fast], math.sqrt(2 * c["tcp_acc"] * dist), self.tcp_speed + c["tcp_acc"] * dt)
        self.tcp_speed = vcap if dist > 1e-5 else 0.0
        move = min(dist, vcap * dt)
        dxz = (self.pend[0] / dist * move, self.pend[1] / dist * move) if dist > 1e-9 else (0.0, 0.0)
        pmax = D(c["joint_max_dps"]["joint_hand"]) * dt
        dp = clamp(self.pend[2], -pmax, pmax)
        q["joint_wrist"] += D(c["roll_dps"][fast]) * it.roll * dt
        if abs(it.grip) > 1e-6:
            self.grip_goal = None
            q["joint_grip_L"] += D(c["grip_dps"][fast]) * it.grip * dt
        elif self.grip_goal is not None:
            g = D(c["grip_dps"][fast]) * dt
            q["joint_grip_L"] += clamp(self.grip_goal - q["joint_grip_L"], -g, g)
            if abs(self.grip_goal - q["joint_grip_L"]) < 1e-4:
                self.grip_goal = None
        ik_used = (0.0, 0.0, 0.0)
        if move > 0 or abs(dp) > 0 or abs(it.roll) > 1e-6:
            t0 = kin.task(self.chain.fk(q0))
            target = (t0[0] + dxz[0], t0[1] + dxz[1], t0[2] + dp)
            r = kin.solve(self.chain, q, target, bounds={n: self.bounds[n] for n in kin.ARM_IK_JOINTS})
            for n in kin.ARM_IK_JOINTS:
                q[n] = r.q[n]
            ik_used = (dxz[0], dxz[1], dp)
            if not r.ok and (r.residual_m > 0.002 or r.residual_rad > D(1.0)):
                out.ik_short = True   # out of reach / at a limit: drop the queued motion, don't wind up
                self.clear_pending()
                self.tcp_speed = 0.0

        # -- 3. joint bounds, speed and acceleration caps (arm)
        for n in ARM:
            lo, hi = self.bounds[n]
            if q[n] < lo or q[n] > hi:
                if q0[n] < lo or q0[n] > hi:   # started outside (anchored there): only allow moving inward
                    q[n] = max(q[n], q0[n]) if q0[n] < lo else min(q[n], q0[n])
                else:
                    q[n] = clamp(q[n], lo, hi)
                out.limited.append(n)
        scale = 1.0
        for n in ARM:
            d = abs(q[n] - q0[n])
            cap = D(c["joint_max_dps"][n]) * dt
            if d > cap:
                scale = min(scale, cap / d)
        for n in ARM:
            v_new = (q[n] - q0[n]) * scale / dt
            a = D(c["joint_max_acc_dps2"][n]) * dt
            v_new = clamp(v_new, self.v_joint[n] - a, self.v_joint[n] + a)
            q[n] = q0[n] + v_new * dt
            lo, hi = self.bounds[n]
            if (q[n] < lo and q[n] < q0[n]) or (q[n] > hi and q[n] > q0[n]):  # bounds beat the accel limit
                q[n] = clamp(q[n], min(lo, q0[n]), max(hi, q0[n]))
                v_new = (q[n] - q0[n]) / dt
            self.v_joint[n] = v_new
        if scale < 1.0:
            out.limited.append("joint speed")

        # -- 4. tracking governor (estimated firmware lag)
        if measured is not None:
            fw = self.fw_target_estimate(now) or q0
            gov = c["governor"]
            lag = {"base_x": (fw["base_x"] - measured["base_x"], gov["base_x"]),
                   "base_y": (fw["base_y"] - measured["base_y"], gov["base_y"]),
                   "base_theta": (fw["base_theta"] - measured["base_theta"], D(gov["base_theta_deg"])),
                   "joint_elbow": (fw["joint_elbow"] - measured["joint_elbow"], D(gov["joint_elbow_deg"])),
                   "joint_shoulder": (fw["joint_shoulder"] - measured["joint_shoulder"], D(gov["joint_shoulder_deg"]))}
            for n, (e, g) in lag.items():
                step = q[n] - q0[n]
                if abs(e) > g and step * e > 0:   # lagging and still pushing further the same way
                    out.governed.append(n)
            if any(n.startswith("base") for n in out.governed):
                for n in ("base_x", "base_y", "base_theta"):
                    q[n] = q0[n]
                self.v_body = [0.5 * v for v in self.v_body]
            if any(n.startswith("joint") for n in out.governed):
                for n in kin.ARM_IK_JOINTS:
                    q[n] = q0[n]
                    self.v_joint[n] = 0.0
                self.tcp_speed = 0.0
                ik_used = (0.0, 0.0, 0.0)
            # far from the command: the robot is not following (E-stop, power, stall, pushed)
            lost = c["lost"]
            if math.hypot(q0["base_x"] - measured["base_x"], q0["base_y"] - measured["base_y"]) > lost["base_xy"]:
                out.lost = "base is not following the command (E-stop? motor power? stuck?)"
            elif abs(q0["base_theta"] - measured["base_theta"]) > D(lost["base_theta_deg"]):
                out.lost = "base heading is not following the command"
            else:
                for n in ("joint_elbow", "joint_shoulder"):
                    if abs(q0[n] - measured[n]) > D(lost["arm_deg"]):
                        out.lost = f"{n[6:]} is {math.degrees(abs(q0[n] - measured[n])):.1f} deg from the command (stall? power?)"
                        break

        moving = any(abs(q[n] - q0[n]) > 1e-7 for n in ORDER)

        # -- 5. collision probe
        if moving and check is not None:
            k = max(1.0, c["probe_sec"] / dt)
            dxy = math.hypot(q["base_x"] - q0["base_x"], q["base_y"] - q0["base_y"])
            dth = abs(q["base_theta"] - q0["base_theta"])
            if dxy > 1e-9:
                k = max(k, c["probe_min_base_m"] / dxy)
            if dth > 1e-9:
                k = max(k, D(c["probe_min_base_deg"]) / dth)
            if any(abs(q[n] - q0[n]) > 1e-9 for n in kin.ARM_IK_JOINTS + ("joint_wrist",)):
                p0, p1 = self.chain.fk(q0)[:3, 3], self.chain.fk(q)[:3, 3]
                dtcp = float(math.dist(p0, p1))
                if dtcp > 1e-9:
                    k = max(k, c["probe_min_tcp_m"] / dtcp)
            k = min(k, 400.0)
            probe = {n: q0[n] + (q[n] - q0[n]) * k for n in ORDER}
            for n in ARM:
                lo, hi = self.bounds[n]
                probe[n] = clamp(probe[n], min(lo, q0[n]), max(hi, q0[n]))
            res = check(probe)
            out.check_ms = res.ms
            links = self.chain.links
            blocking = filter_contacts(res.contacts, c["collision_mode"], links)
            grip_only = all(abs(q[n] - q0[n]) <= 1e-9 for n in ORDER if n != "joint_grip_L")
            if grip_only:
                fingers = set(c["finger_links"])
                blocking = [x for x in blocking if not (x[0] in fingers or x[1] in fingers)]
            invalid = (not res.valid) and (bool(blocking) or not res.contacts or res.error is not None)
            if invalid:
                why = res.error or (", ".join(f"{a} / {b}" for a, b, _ in blocking[:3]) or "invalid state")
                escape = False
                if res.error is None and blocking:
                    cur = check(q0)
                    cur_block = filter_contacts(cur.contacts, c["collision_mode"], links) if cur.error is None else None
                    if cur_block:
                        depth_cur = {(a, b): d for a, b, d in cur_block}
                        escape = all((a, b) in depth_cur and d <= depth_cur[(a, b)] + 1e-4 for a, b, d in blocking)
                if escape:
                    out.escape = True
                    out.contacts = blocking
                else:
                    q = dict(q0)
                    moving = False
                    self.v_body = [0.0, 0.0, 0.0]
                    self.v_joint = {n: 0.0 for n in ARM}
                    self.tcp_speed = 0.0
                    self.clear_pending()
                    self.grip_goal = None
                    out.blocked = why
                    out.contacts = blocking
                    ik_used = (0.0, 0.0, 0.0)

        # consume what was actually executed (speed caps may have shortened the IK step)
        if moving and not out.blocked and any(ik_used):
            t0, t1 = kin.task(self.chain.fk(q0)), kin.task(self.chain.fk(q))
            n2 = ik_used[0] ** 2 + ik_used[1] ** 2
            if n2 > 0:
                f = clamp(((t1[0] - t0[0]) * ik_used[0] + (t1[1] - t0[1]) * ik_used[1]) / n2, 0.0, 1.0)
                self.pend[0] -= ik_used[0] * f
                self.pend[1] -= ik_used[1] * f
            if ik_used[2]:
                self.pend[2] -= ik_used[2] * clamp((t1[2] - t0[2]) / ik_used[2], 0.0, 1.0)
        self.q = q
        self.history.append((now, dict(q)))
        out.q = dict(q)
        out.moving = moving
        return out
