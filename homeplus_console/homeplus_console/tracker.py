"""The few things the console must remember because they can't be re-observed (UI_PLAN §3.1):
the odometry origin, bridge sessions, sticky faults, acknowledged aborts, motor-power evidence,
operator declarations and the detection-cycle window. Everything else is derived fresh each tick.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

from .signals import SignalStore

STICKY_CODES = ("TRACKING_ERROR", "SETTLE_TIMEOUT", "STALE_FEEDBACK", "RX_ACK_TIMEOUT", "HOST_TIMEOUT")

# Suggested check per Arduino error code (UI_PLAN §4.3).
ERROR_HINTS = {
    "TRACKING_ERROR": "Check the E-stop, the arm power switch and the battery clips.",
    "SETTLE_TIMEOUT": "Arm didn't settle: check the E-stop, power switch and battery clips.",
    "STALE_FEEDBACK": "Bridge lost firmware telemetry: check the Arduino USB cable.",
    "RX_ACK_TIMEOUT": "Arduino didn't acknowledge: check USB, then reconnect the Arduino.",
    "HOST_TIMEOUT": "Firmware watchdog fired: the host link stalled.",
    "FIRMWARE_RESTART": "The Arduino rebooted: odometry is now zero wherever the robot stands.",
    "HOST_STOP": "Stopped by the console.",
}


def error_code(ack: str) -> Optional[str]:
    parts = ack.split()
    if len(parts) >= 3 and parts[0] == "ERROR":
        return parts[2]
    return None


@dataclass
class Memory:
    origin: Dict[str, Any] = field(default_factory=lambda: {
        "status": "unconfirmed", "at": None, "by": None, "session_id": None, "note": None,
        "lost_reason": None, "lost_at": None, "base_at_reset": None})
    session: Dict[str, Any] = field(default_factory=lambda: {
        "id": None, "start_mono": None, "start_wall": None, "upright_reached": False, "upright_at": None})
    last_joints: Optional[Dict[str, float]] = None
    held: Optional[Dict[str, float]] = None
    faults: List[Dict[str, Any]] = field(default_factory=list)
    abort_acked_t: Optional[float] = None
    motor_verified: Optional[Dict[str, Any]] = None
    estop_declaration: Optional[Dict[str, Any]] = None
    declared: Dict[str, Optional[Dict[str, Any]]] = field(default_factory=lambda: {
        "one_target": None, "area_clear": None, "at_start_spot": None})
    cycles: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=30))
    rezero_expected: bool = False    # a set-start flow is resetting the Arduino on purpose
    zero_expected: bool = False      # a set-start flow sent ZERO on purpose
    state_onsets: Dict[str, float] = field(default_factory=dict)
    last_result_seen_t: Optional[float] = None
    zero_supported: bool = False
    tf_drops: Deque[float] = field(default_factory=lambda: deque(maxlen=500))


class Tracker:
    def __init__(self, store: SignalStore, data_dir: Path, preconfirm_origin: bool = False):
        self.store = store
        self.mem = Memory()
        self.data_dir = data_dir
        self._event_seq = 0
        self._preconfirm = preconfirm_origin
        self._persisted = self._load_origin()

    # ------------------------------------------------------------------ persistence
    def _origin_path(self) -> Path:
        return self.data_dir / "origin.json"

    def _load_origin(self) -> Optional[Dict[str, Any]]:
        try:
            return json.loads(self._origin_path().read_text())
        except (OSError, ValueError):
            return None

    def _save_origin(self) -> None:
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self._origin_path().write_text(json.dumps(self.mem.origin))

    # ------------------------------------------------------------------ operator inputs
    def confirm_origin(self, by: str, note: str) -> None:
        o = self.mem.origin
        o.update(status="confirmed", at=time.time(), by=by, session_id=self.mem.session["id"], note=note,
                 lost_reason=None, lost_at=None, base_at_reset=None)
        self.mem.rezero_expected = False
        self.mem.zero_expected = False
        self.mem.held = dict(self.mem.last_joints or {})
        self._save_origin()

    def acknowledge(self, by: str) -> List[str]:
        """Acknowledge every sticky fault and the last abort."""
        done = []
        for f in self.mem.faults:
            if not f["acknowledged"]:
                f.update(acknowledged=True, ack_by=by, ack_at=time.time())
                done.append(f["code"])
        es = self.store.get("/gesture_control/executor_state")
        lr = (es.value or {}).get("last_result") if es else None
        if lr and lr.get("t"):
            self.mem.abort_acked_t = lr["t"]
            done.append("abort")
        for k in self.mem.declared:
            self.mem.declared[k] = None
        return done

    # ------------------------------------------------------------------ observation
    def observe(self, now: float) -> None:
        m = self.mem
        for seq, name, value, t in self.store.events_since(self._event_seq):
            self._event_seq = seq
            if name == "/arduino/bridge_info":
                self._new_session(value, t)
            elif name == "/arduino/ack":
                self._ack(value)
            elif name == "/molmopoint/diagnostics":
                m.cycles.append({"t": t, "stage": value.get("stage"), "reason": value.get("reason"),
                                 "detail": value.get("detail"), "pose": None, "task_id": value.get("task_id")})
            elif name == "tf/drop":
                m.tf_drops.append(t)
            elif name.startswith("/molmopoint/task_") and name.endswith("/pose"):
                if m.cycles and m.cycles[-1]["pose"] is None and t - m.cycles[-1]["t"] < 1.0:
                    m.cycles[-1]["pose"] = (value["x"], value["y"], value["z"])
        js = self.store.get("/hardware/joint_states")
        if js and js.value and js.age(now) is not None and js.age(now) < 1.0:
            j = js.value
            prev = m.last_joints
            # Reset evidence without a banner: odometry snapping to exactly zero from > 2 cm away.
            if prev and not (m.rezero_expected or m.zero_expected) and m.origin["status"] == "confirmed" \
                    and math.hypot(prev["base_x"], prev["base_y"]) > 0.02 \
                    and abs(j["base_x"]) < 1e-4 and abs(j["base_y"]) < 1e-4 and abs(j["base_theta"]) < 1e-3:
                self._lose_origin("Odometry jumped to (0, 0, 0): the Arduino was reset or zeroed outside the console", prev)
            m.last_joints = dict(j)
            if m.held is None and m.session["id"] and not m.session.get("observed_boot", True):
                m.held = dict(j)
            s = m.session
            still = self.store.get("/arduino/status")
            if s["id"] and not s["upright_reached"] and abs(j["joint_elbow"]) < 3.0 and abs(j["joint_shoulder"]) < 3.0 \
                    and still is not None and still.value is True and still.recv_mono >= s["start_mono"]:
                s["upright_reached"] = True
                s["upright_at"] = time.time()
                m.motor_verified = {"at": time.time(), "how": "arm rose to upright after bridge start"}
                m.held = dict(j)
        es = self.store.get("/gesture_control/executor_state")
        if es and es.value:
            lr = es.value.get("last_result")
            if lr and lr.get("t") != m.last_result_seen_t:
                m.last_result_seen_t = lr.get("t")
                if lr.get("outcome") in ("aborted", "failed", "cancelled") or lr.get("outcome") == "succeeded":
                    # declarations are per run (§5.2)
                    for k in m.declared:
                        m.declared[k] = None

    def _lose_origin(self, reason: str, base: Optional[Dict[str, float]]) -> None:
        self.mem.origin.update(status="lost", lost_reason=reason, lost_at=time.time(), session_id=None,
                               base_at_reset={k: base.get(k) for k in ("base_x", "base_y", "base_theta")} if base else None)
        self._save_origin()

    def _new_session(self, info: Dict[str, Any], t: float) -> None:
        """Origin and motor-power evidence are tied to a firmware boot (boot_id), not to a bridge session:
        with the serial bridge, restarting the ROS bridge reopens a pty and does not reset the Mega."""
        m = self.mem
        if "boot_id" in info and info["boot_id"] is None:
            return  # a bridge that reports boots hasn't heard this one's BOOT line yet: no new identity
        sid = info.get("boot_id") or info.get("session_id")
        m.zero_supported = bool(info.get("zero_supported"))
        if sid == m.session["id"]:
            return
        first = m.session["id"] is None
        prev_base = None
        if m.last_joints:
            prev_base = {k: m.last_joints.get(k) for k in ("base_x", "base_y", "base_theta")}
        # Only a boot the console saw happen proves anything about the boot-upright rule; a robot that
        # booted before the console started may have moved its arm since.
        m.session = {"id": sid, "start_mono": t, "start_wall": time.time(), "upright_reached": False, "upright_at": None,
                     "observed_boot": not first}
        m.motor_verified = None
        # A boot the console saw zeroes odometry and the held target. For one it didn't see, the held
        # target is unknown: the first fresh pose becomes the baseline for the drift rule.
        m.held = {"base_x": 0.0, "base_y": 0.0, "base_theta": 0.0} if not first else None
        if first:
            if self._persisted and self._persisted.get("session_id") == sid and self._persisted.get("status") == "confirmed":
                m.origin = dict(self._persisted)
            elif self._preconfirm:
                m.origin.update(status="confirmed", at=time.time(), by="sim", session_id=sid,
                                note="sim start: robot placed at the start spot")
            return
        if m.rezero_expected:
            return  # the guided flow owns this reset
        self._lose_origin("Arduino reset observed (firmware rebooted)", prev_base)

    def _ack(self, ack: str) -> None:
        m = self.mem
        if ack.startswith("ZEROED"):
            if not m.zero_expected:
                self._lose_origin("Odometry zeroed (ZERO) outside the console", m.last_joints)
            return
        if ack.startswith("DONE"):
            if m.last_joints:
                m.held = dict(m.last_joints)
            m.motor_verified = {"at": time.time(), "how": "last command reached its target"}
            return
        code = error_code(ack)
        if code in STICKY_CODES:
            m.faults.append({"id": len(m.faults) + 1, "code": code, "text": ack, "onset": time.time(),
                             "acknowledged": False, "hint": ERROR_HINTS.get(code, "")})
        if code == "FIRMWARE_RESTART" and not m.rezero_expected:
            self._lose_origin("Arduino firmware restarted (FIRMWARE_RESTART)", m.last_joints)


def finite(v: Any) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)
