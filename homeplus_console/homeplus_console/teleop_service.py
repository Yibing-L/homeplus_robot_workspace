"""Teleop mode: browser input → Controller → bridge jog stream, with the dead-man rules.

Transport: the browser sends one input frame every ~33 ms on the console's existing WebSocket
(type "teleop"), only while teleop mode is on, this tab holds control, and the tab is focused and
visible. This thread runs at 25 Hz and publishes one absolute setpoint per tick on /arduino/jog.

Phases:
  off      teleop mode off
  ready    mode on, nothing streaming; the first non-zero input opens a jog session
  starting first point sent at the measured pose; waiting for the bridge to accept it
  active   streaming setpoints
  ending   stream finished; the robot holds its last point and the firmware reports DONE
  latched  stopped (dead-man, STOP, collision service down, bridge error): release every key and
           mouse button with the tab focused to return to ready

Stops (each one ends the stream; the robot halts at most one bridge lookahead, 0.3 s, later):
  - no input frame for deadman_sec (0.3 s), the tab loses focus or is hidden, control lost;
  - the bridge sees no setpoint for 0.15 s (console stalled) and ends the stream itself;
  - the firmware runs out of points (BUFFER_UNDERRUN) 0.3 s after the last one (bridge dead);
  - STOP / Space: /arduino/stop, the firmware halts immediately (unchanged STOP path).
"""
from __future__ import annotations

import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from . import robot_model
from .recorder import Recorder
from .teleop import ORDER, Check, Controller, Intent, StepOut, merged
from .teleop_kin import ArmChain

FEEDBACK_FRESH_SEC = 0.3


class TeleopService:
    def __init__(self, io, store, tracker, logs, cfg: Dict[str, Any], data_dir: Path, clock=time.monotonic):
        self.io, self.store, self.tracker, self.logs, self.clock = io, store, tracker, logs, clock
        self.cfg = merged(cfg.get("teleop"))
        xml = robot_model.urdf_xml()
        self.chain = ArmChain(xml) if xml else None
        self.ctrl = Controller(self.chain, self.cfg) if self.chain else None
        self.recorder = Recorder(data_dir / "teleop_episodes", io, fps=float(cfg.get("teleop_record_fps", 10.0)), logs=logs)
        self.lock = threading.RLock()
        self.mode = False
        self.owner: Optional[str] = None
        self.operator: Optional[str] = None
        self.phase = "off"
        self.reason: Optional[str] = None
        self.reason_at: Optional[float] = None
        self.session: Optional[str] = None
        self.t_phase = 0.0
        self.intent = Intent()
        self.last_frame: Optional[float] = None
        self.last_seq = -1
        self.focused = False
        self.mouse_acc = [0.0, 0.0, 0.0]
        self.grip_toggles = 0
        self.idle_since: Optional[float] = None
        self._ending_session: Optional[str] = None
        self.last_out: Optional[StepOut] = None
        self.last_contacts: List = []
        self.blocked_at: Optional[float] = None
        self.check_errors = 0
        self.frame_gaps: List[float] = []
        self.state_getter: Callable[[], Optional[Dict[str, Any]]] = lambda: None
        self.lease_ok: Callable[[str, float], bool] = lambda client, now: False
        self._ev = max((e[0] for e in store.events_since(0)), default=0)
        self._stop = threading.Event()

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        threading.Thread(target=self._loop, name="teleop", daemon=True).start()

    def _loop(self) -> None:
        period = 1.0 / float(self.cfg["rate_hz"])
        last = self.clock()
        while not self._stop.is_set():
            t0 = time.monotonic()
            now = self.clock()
            try:
                self.tick(now, now - last)
            except Exception as exc:  # noqa: BLE001 — never let a bug leave a stream running
                self.logs.add("console", "ERROR", f"teleop tick failed: {exc!r}")
                with self.lock:
                    self._latch(f"internal error: {exc!r}", now)
            last = now
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # ------------------------------------------------------------------ operator
    def enable(self, client: str, operator: str, now: float) -> Dict[str, Any]:
        with self.lock:
            if self.ctrl is None:
                return {"status": "rejected", "detail": "robot URDF not found: teleop needs it for the arm kinematics"}
            self.mode, self.owner, self.operator = True, client, operator
            self._set_phase("latched", now, "release every key and mouse button with this tab focused to start")
            self.io.set_feed_boost(True)
            self.logs.add("console", "WARN", f"Teleop mode ON ({operator})")
            return {"status": "confirmed", "detail": "teleop mode on"}

    def disable(self, reason: str, now: float) -> None:
        with self.lock:
            if not self.mode:
                return
            self._end_session("teleop off")
            if self.recorder.active:
                self.recorder.stop("aborted", reason)
            self.mode, self.owner = False, None
            self._set_phase("off", now, reason)
            self.io.set_feed_boost(False)
            self.logs.add("console", "WARN", f"Teleop mode OFF: {reason}")

    def on_stop(self, now: float) -> None:
        """Console STOP (button or Space): the bridge stops the firmware; we latch."""
        with self.lock:
            if self.mode:
                self.session = None   # the bridge failed the stream with HOST_STOP
                self._set_phase("latched", now, "STOP pressed")

    def set_collision_mode(self, mode: str) -> Dict[str, Any]:
        if mode not in ("full", "no_octomap", "self_only"):
            return {"status": "rejected", "detail": f"unknown collision mode {mode}"}
        with self.lock:
            self.ctrl.cfg["collision_mode"] = mode
            self.logs.add("console", "WARN", f"Teleop collision check: {mode}")
        return {"status": "confirmed", "detail": f"collision check: {mode}"}

    def on_frame(self, client: Optional[str], msg: Dict[str, Any], now: float) -> None:
        """One input frame from the browser (WebSocket thread). Cheap: just store it."""
        with self.lock:
            if not self.mode or client is None or client != self.owner:
                return
            seq = int(msg.get("seq", 0))
            if seq <= self.last_seq and seq != 0:
                return
            self.last_seq = seq
            if self.last_frame is not None:
                self.frame_gaps.append(now - self.last_frame)
                self.frame_gaps = self.frame_gaps[-50:]
            self.last_frame = now
            self.focused = bool(msg.get("focused")) and bool(msg.get("visible", True))
            self.intent = Intent.parse(msg.get("intent") or {})
            m = msg.get("mouse") or {}
            try:
                dx, dy, wh = float(m.get("dx", 0)), float(m.get("dy", 0)), float(m.get("wheel", 0))
            except (TypeError, ValueError):
                dx = dy = wh = 0.0
            if all(math.isfinite(v) for v in (dx, dy, wh)) and self.focused:
                self.mouse_acc = [max(-2000.0, min(2000.0, a + b)) for a, b in zip(self.mouse_acc, (dx, dy, wh))]
            self.grip_toggles += int(msg.get("grip_toggle", 0) or 0) if self.focused else 0

    # ------------------------------------------------------------------ helpers
    def _set_phase(self, phase: str, now: float, reason: Optional[str] = None) -> None:
        self.phase, self.t_phase = phase, now
        if reason is not None:
            self.reason, self.reason_at = reason, time.time()

    def _latch(self, reason: str, now: float) -> None:
        if not self.mode:
            return
        self._end_session(reason)
        self._set_phase("latched", now, reason)
        self.mouse_acc = [0.0, 0.0, 0.0]
        self.grip_toggles = 0
        if self.ctrl:
            self.ctrl.reset_motion()

    def _end_session(self, why: str) -> None:
        if self.session is not None:
            self.io.jog_end(self.session)
            self.logs.add("console", "INFO", f"Teleop stream end: {why}")
            self.session = None

    def _measured(self, now: float) -> Optional[Dict[str, float]]:
        js = self.store.get("/hardware/joint_states_si")
        if js is None or js.value is None or js.age(now) is None or js.age(now) > FEEDBACK_FRESH_SEC:
            return None
        v = js.value
        return {n: float(v[n]) for n in ORDER} if all(n in v for n in ORDER) else None

    def _check(self, q: Dict[str, float]) -> Check:
        res = self.io.check_state(q)
        if res.error:
            self.check_errors += 1
        else:
            self.check_errors = 0
        return res

    def _drain_jog_events(self, now: float) -> None:
        for seq, name, value, _t in self.store.events_since(self._ev):
            self._ev = seq
            if name != "/arduino/jog_status" or not isinstance(value, dict):
                continue
            sid = value.get("session")
            if sid is None or (sid != self.session and not (self.phase == "ending" and sid == self._ending_session)):
                continue
            st, why = value.get("state"), value.get("reason") or ""
            if st == "active" and self.phase == "starting":
                self._set_phase("active", now, None)
                self.idle_since = None
            elif st == "rejected":
                self.session = None
                self._latch(f"bridge refused to start: {why}", now)
            elif st == "failed":
                self.session = None
                self._latch(f"bridge stopped the stream: {why}", now)
            elif st == "ending" and self.phase == "active" and why != "console end":
                # the bridge finished the stream itself (our setpoints stopped arriving, or 110 s limit)
                self._ending_session = self.session
                self.session = None
                self._set_phase("ending", now, f"bridge ended the stream: {why}")
            elif st == "done":
                self.session = None
                if self.phase in ("ending", "active", "starting"):
                    self._set_phase("ready", now, None)

    # ------------------------------------------------------------------ tick
    def tick(self, now: float, dt: float) -> None:
        with self.lock:
            self._drain_jog_events(now)
            if not self.mode:
                return
            st = self.state_getter()
            if not self.lease_ok(self.owner, now):
                self.disable("control lost or released", now)
                return
            if st is not None:
                ex = st["executor"]
                if ex.get("busy"):
                    self.disable("the executor started a command", now)
                    return
                if any(not f["acknowledged"] for f in st["faults"]):
                    self.disable("hardware fault: " + ", ".join(f["code"] for f in st["faults"] if not f["acknowledged"]), now)
                    return
            measured = self._measured(now)
            frame_age = None if self.last_frame is None else now - self.last_frame
            deadman = frame_age is not None and frame_age <= self.cfg["deadman_sec"] and self.focused
            it = self.intent if deadman else Intent()
            wants = it.any() or any(abs(v) > 0 for v in self.mouse_acc) or self.grip_toggles > 0

            if self.phase in ("starting", "active"):
                if not deadman:
                    why = ("tab lost focus" if frame_age is not None and frame_age <= self.cfg["deadman_sec"]
                           else f"no input for {frame_age * 1000:.0f} ms" if frame_age is not None else "no input")
                    self._latch(f"dead-man: {why}", now)
                elif measured is None:
                    self._latch("joint feedback stale", now)
                elif self.check_errors >= 3:
                    self._latch("collision check unavailable (move_group /check_state_validity)", now)

            if self.phase == "latched":
                if deadman and not it.any() and self.session is None:
                    self.mouse_acc, self.grip_toggles = [0.0, 0.0, 0.0], 0
                    self._set_phase("ready", now, None)
            elif self.phase == "ready":
                if deadman and wants and measured is not None:
                    ready = st is not None and st["joints"].get("ready") is True
                    if not ready:
                        self.reason, self.reason_at = "waiting for the bridge to report ready (robot still)", time.time()
                    else:
                        q = self.ctrl.anchor(measured, now)
                        self.session = uuid.uuid4().hex[:10]
                        self.io.jog_point(self.session, [q[n] for n in ORDER])
                        self._set_phase("starting", now, None)
            elif self.phase == "starting":
                self.io.jog_point(self.session, [self.ctrl.q[n] for n in ORDER])   # hold the anchor
                if now - self.t_phase > 1.5:
                    self._latch("bridge did not start the stream within 1.5 s (bridge without jog support?)", now)
            elif self.phase == "active":
                dx, dy, wh = self.mouse_acc
                self.mouse_acc = [0.0, 0.0, 0.0]
                self.ctrl.add_mouse(dx, dy, wh, it.fast)
                for _ in range(min(self.grip_toggles, 1)):
                    self.ctrl.toggle_grip()
                self.grip_toggles = 0
                out = self.ctrl.step(now, dt, it, measured, self._check)
                self.last_out = out
                if out.blocked:
                    self.blocked_at = now
                    self.last_contacts = out.contacts
                if out.lost:
                    self._latch(out.lost, now)
                else:
                    self.io.jog_point(self.session, [out.q[n] for n in ORDER])
                    if not it.any() and not self.ctrl.busy():
                        self.idle_since = self.idle_since or now
                        if now - self.idle_since >= self.cfg["idle_end_sec"]:
                            self._ending_session = self.session
                            self._end_session("idle")
                            self._set_phase("ending", now, None)
                    else:
                        self.idle_since = None
            elif self.phase == "ending":
                if now - self.t_phase > 8.0:
                    self._latch("stream end not confirmed (no DONE in 8 s)", now)

            if self.phase in ("starting", "active", "ending") and self.tracker.mem.last_joints:
                # commanded motion, not a push: keep the console's drift rule quiet
                self.tracker.mem.held = dict(self.tracker.mem.last_joints)
            if self.recorder.active:
                self.recorder.sample(now, measured, self.ctrl.q if self.phase == "active" else None,
                                     self.ctrl.v_body if self.ctrl else None, it)

    # ------------------------------------------------------------------ recording
    def record_start(self, instruction: str, now: float) -> Dict[str, Any]:
        with self.lock:
            if not self.mode:
                return {"status": "rejected", "detail": "turn teleop mode on first"}
            if not instruction.strip():
                return {"status": "rejected", "detail": "type the language instruction first"}
            path = self.recorder.start(instruction.strip(), self.operator or "operator")
            return {"status": "confirmed", "detail": f"recording to {path}"}

    def record_stop(self, outcome: str) -> Dict[str, Any]:
        with self.lock:
            if not self.recorder.active:
                return {"status": "rejected", "detail": "not recording"}
            info = self.recorder.stop(outcome)
            return {"status": "confirmed", "detail": info}

    # ------------------------------------------------------------------ view
    def view(self, now: float) -> Dict[str, Any]:
        with self.lock:
            c = self.ctrl
            out = self.last_out
            tcp = c.tcp() if c and c.q else None
            gaps = sorted(self.frame_gaps)
            return {
                "mode": self.mode, "phase": self.phase, "owner_label": self.operator if self.mode else None,
                "reason": self.reason, "reason_wall": self.reason_at,
                "session": self.session,
                "frame_age_ms": None if self.last_frame is None else round((now - self.last_frame) * 1000),
                "frame_gap_p95_ms": round(gaps[int(0.95 * (len(gaps) - 1))] * 1000) if gaps else None,
                "focused": self.focused, "keys": list(self.intent.keys), "fast": self.intent.fast,
                "base_vel": None if not c else {"vx": round(c.v_body[0], 3), "vy": round(c.v_body[1], 3),
                                                 "wz_deg": round(math.degrees(c.v_body[2]), 1)},
                "tcp": None if tcp is None else {"reach": round(tcp[0], 3), "height": round(tcp[1], 3),
                                                  "pitch_deg": round(math.degrees(tcp[2]), 1)},
                "tcp_speed": None if not c else round(c.tcp_speed, 3),
                "joint_vel_deg": None if not c else {k: round(math.degrees(v), 1) for k, v in c.v_joint.items()},
                "command": None if not (c and c.q) else {k: (round(v, 4) if k in ("base_x", "base_y") else round(math.degrees(v), 2))
                                                         for k, v in c.q.items()},
                "blocked": None if not out or not out.blocked else out.blocked,
                "blocked_age": None if self.blocked_at is None else round(now - self.blocked_at, 1),
                "contacts": [[a, b, round(d, 4)] for a, b, d in (self.last_contacts or [])[:4]],
                "escape": bool(out and out.escape), "governed": list(out.governed) if out else [],
                "limited": list(out.limited) if out else [], "ik_short": bool(out and out.ik_short),
                "check_ms": None if not out or out.check_ms is None else round(out.check_ms, 1),
                "check_errors": self.check_errors,
                "collision_mode": c.cfg["collision_mode"] if c else None,
                "limits": {"base_mps": self.cfg["base_lin_mps"], "base_dps": self.cfg["base_rot_dps"],
                           "tcp_mps": self.cfg["tcp_mps"], "deadman_ms": round(self.cfg["deadman_sec"] * 1000)},
                "jog_capable": self.io.jog_capable(),
                "recording": self.recorder.view(),
            }
