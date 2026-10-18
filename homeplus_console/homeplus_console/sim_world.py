"""Signal-level simulation of the HomePlus robot, for developing the console without ROS.

It writes the same signal names the rclpy layer will write (topic names from UI_PLAN §2),
with the same semantics: latched /arduino/ready every 5 s, /arduino/status on change only,
an Arduino reset zeroing base odometry, the firmware commanding the arm upright at boot,
the detector reporting stage/reason per cycle, and an executor that follows the
pick_up_coffee step list from gesture_execution_map.yaml.

Faults (set via /api/sim/fault) reproduce the failure modes in UI_PLAN_PROMPT.md.
Everything runs in one tick thread, so the sim is deterministic apart from wall time.
"""
from __future__ import annotations

import math
import random
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from . import sim_render
from .images import FrameStore
from .log_filter import LogBuffer
from .signals import SignalStore

TICK = 0.02
FOLDED = {"joint_elbow": -35.0, "joint_shoulder": -30.0}
CUP = (0.95, 0.02, 0.762)          # cup in world (start spot = origin), metres
SECOND_OBJECT = (0.95, 0.42, 0.765)

EXECUTOR = "gesture_motion_executor"
BRIDGE = "arduino_bridge"
DETECTOR = "molmopoint"

# pick_up_coffee from gesture_execution_map.yaml (+ the initial detection step).
PICKUP_STEPS = [
    ("detection", "detect", False, {}),
    ("open_gripper", "gripper", False, {"joint_grip_L": 40.0}),
    ("retract_arm_for_pickup", "joint_state", False, {"joint_shoulder": 12.0, "joint_elbow": 24.0, "joint_wrist": 0.0}),
    ("move_to_pregrasp", "pose", False, {"base_x": 0.42, "joint_shoulder": 30.0, "joint_elbow": 60.0}),
    ("refine_target", "detect", False, {}),
    ("align_refined_grasp", "pose", True, {"base_dx": 0.03, "base_dy": -0.01, "base_dtheta": 1.2,
                                           "joint_shoulder": 38.0, "joint_elbow": 71.0}),
    ("move_to_grasp", "pose", False, {"base_dx": 0.05, "joint_shoulder": 42.0, "joint_elbow": 78.0}),
    ("close_gripper_at_target", "gripper", False, {"joint_grip_L": 5.0}),
    ("lift", "pose", False, {"joint_shoulder": 30.0}),
]
BUTTON_STEPS = [
    ("detection", "detect", False, {}),
    ("move_to_prebutton", "pose", False, {"base_x": 0.40, "joint_shoulder": 28.0, "joint_elbow": 55.0}),
    ("align_button", "pose", True, {"base_dx": 0.02, "joint_shoulder": 34.0, "joint_elbow": 64.0}),
    ("press_button", "pose", False, {"base_dx": 0.04}),
    ("retract_from_button", "pose", False, {"base_dx": -0.10, "joint_shoulder": 10.0, "joint_elbow": 20.0}),
]
COMMANDS = {
    "pick_up_coffee": ("sequence", PICKUP_STEPS, 1),
    "press_square_button": ("sequence", BUTTON_STEPS, 2),
    "home": ("joint_state", [("home", "joint_state", True, {"base_x": 0.0, "base_y": 0.0, "base_theta": 0.0,
                                                              "joint_shoulder": 0.0, "joint_elbow": 0.0,
                                                              "joint_wrist": 0.0})], None),
    "gripper_open": ("gripper", [("gripper_open", "gripper", False, {"joint_grip_L": 40.0})], None),
    "gripper_close": ("gripper", [("gripper_close", "gripper", False, {"joint_grip_L": 5.0})], None),
}

HOME_TARGETS = {"base_x": 0.0, "base_y": 0.0, "base_theta": 0.0, "joint_shoulder": 0.0, "joint_elbow": 0.0,
                "joint_wrist": 0.0}
RECOVERY = ("return_to_start", "undo_step", "undo_sequence", "resume")

ARM = ("joint_elbow", "joint_shoulder", "joint_wrist", "joint_hand", "joint_grip_L")
BASE = ("base_x", "base_y", "base_theta")


class SimSupervisor:
    """Stands in for systemd --user units. Same fields as `systemctl show`."""

    units = ["hp-tf", "hp-camera-rx", "hp-vision", "hp-moveit", "hp-bridge"]

    def __init__(self, clock, logs: LogBuffer):
        self.clock = clock
        self.logs = logs
        self._lock = threading.Lock()
        self._u: Dict[str, Dict[str, Any]] = {
            u: {"active_state": "inactive", "sub_state": "dead", "pid": None, "start_mono": None,
                "restarts": 0, "result": "success", "_ready_at": None} for u in self.units}

    def status(self) -> Dict[str, Dict]:
        now = self.clock()
        out = {}
        with self._lock:
            for name, u in self._u.items():
                if u["active_state"] == "activating" and u["_ready_at"] and now >= u["_ready_at"]:
                    u.update(active_state="active", sub_state="running")
                out[name] = {k: v for k, v in u.items() if not k.startswith("_")}
                out[name]["uptime_s"] = (now - u["start_mono"]) if u["start_mono"] else None
        return out

    def is_active(self, unit: str) -> bool:
        return self.status()[unit]["active_state"] == "active"

    def start(self, unit: str) -> Dict:
        with self._lock:
            u = self._u[unit]
            if u["active_state"] in ("active", "activating"):
                return {"result": "already_running", "pid": u["pid"]}
            now = self.clock()
            u.update(active_state="activating", sub_state="start", pid=random.randint(2000, 60000),
                     start_mono=now, _ready_at=now + 1.2)
        self.logs.add("console", "INFO", f"systemd-run --user --unit={unit}: started")
        return {"result": "started"}

    def stop(self, unit: str) -> Dict:
        with self._lock:
            u = self._u[unit]
            u.update(active_state="inactive", sub_state="dead", pid=None, start_mono=None, _ready_at=None)
        self.logs.add("console", "INFO", f"systemctl --user stop {unit}: stopped")
        return {"result": "stopped"}

    def restart(self, unit: str) -> Dict:
        self.stop(unit)
        with self._lock:
            self._u[unit]["restarts"] += 1
        return self.start(unit)

    def fail(self, unit: str, result: str = "exit-code") -> None:
        with self._lock:
            self._u[unit].update(active_state="failed", sub_state="failed", pid=None, result=result)


@dataclass
class ArduinoCmd:
    id: int
    targets: Dict[str, float]
    t_start: float
    kind: str  # "arm" | "base" | "both" | "gripper"
    result: Optional[str] = None  # "DONE" | "ERROR <reason>"


@dataclass
class Faults:
    motor_power: bool = True        # False: E-stop pressed, power switch off or battery clip loose
    usb_connected: bool = True
    sender_alive: bool = True       # mac_sender.py (Mac profile) / realsense node
    sender_frozen: bool = False     # hung sender republishing the same frame
    d455_alive: bool = True
    detector_scenario: str = "good"  # good | two_targets | none | stale_frames | garbage_z | high_first
    gpu_up: bool = True
    zero_supported: bool = True      # firmware/bridge ZERO command (§7.10)


@dataclass
class ExecStep:
    name: str
    type: str
    requires_approval: bool
    targets: Dict[str, float]
    status: str = "pending"
    t_start: Optional[float] = None
    t_end: Optional[float] = None
    note: str = ""
    auto_approved: bool = False
    undo_of: Optional[Dict[str, Any]] = None     # the undo record this step reverses
    start_joints: Optional[Dict[str, float]] = None


class SimWorld:
    mode = "sim"
    structured = True

    def __init__(self, store: SignalStore, frames: FrameStore, logs: LogBuffer, clock=time.monotonic,
                 serial_bridge: bool = True):
        self.store, self.frames, self.logs, self.clock = store, frames, logs, clock
        # Mac profile: tools/arduino_serial_bridge (pty ↔ TCP ↔ socat on the Mac). Restarting the ROS bridge
        # reopens the pty, not the USB device, so it doesn't reset the Mega. Laptop profile: direct USB.
        self.serial_bridge = serial_bridge
        self.port = "$HOME/ttyArduino" if serial_bridge else "/dev/ttyACM0"
        self.boot_id: Optional[str] = None
        self.holding_cup = False
        self.undo_stack: List[Dict[str, Any]] = []
        self._undo_seq = 0
        self._last_sequence: Optional[Dict[str, Any]] = None
        self.supervisor = SimSupervisor(clock, logs)
        self.faults = Faults()
        self._lock = threading.RLock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # physical robot
        self.j: Dict[str, float] = {"base_x": 0.0, "base_y": 0.0, "base_theta": 0.0,
                                    "joint_elbow": FOLDED["joint_elbow"], "joint_shoulder": FOLDED["joint_shoulder"],
                                    "joint_wrist": 0.0, "joint_hand": 0.0, "joint_grip_L": 5.0}
        self.tgt: Dict[str, float] = dict(self.j)
        self.vel: Dict[str, float] = {k: 0.0 for k in self.j}
        # bridge
        self.session_id: Optional[str] = None
        self.session_start: Optional[float] = None
        self.bridge_up = False
        self._last_ready_pub = -99.0
        self._last_status: Optional[bool] = None
        self._last_retry_log = -99.0
        self._next_cmd_id = 1
        self.cmd: Optional[ArduinoCmd] = None
        self._was_bridge_active = False
        # executor
        self.exec: Dict[str, Any] = self._idle_exec()
        self._exec_seq = 0
        self._next_plan_id = 17
        self._approval_response: Optional[bool] = None
        self._was_moveit_active = False
        # detector
        self.task_id = 1
        self._last_cycle = -99.0
        self._cycle_n = 0
        self._still_since: Optional[float] = None
        self.detections: List[Dict[str, Any]] = []
        self._last_debug: Optional[Any] = None
        # cameras
        self._last_frame = -99.0
        self._frozen_color = None
        self._frozen_depth = None
        self._last_d455 = -99.0
        self._last_tf = -99.0
        self._last_moveit = -99.0
        self._last_exec_pub = -99.0
        self._last_gpu = -99.0
        self._last_task_poll = -99.0

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        for u in ("hp-tf", "hp-camera-rx", "hp-vision", "hp-moveit", "hp-bridge"):
            self.supervisor.start(u)
        self._thread = threading.Thread(target=self._run, name="sim-world", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_evt.set()

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                with self._lock:
                    self.tick(self.clock())
            except Exception as exc:  # keep the sim alive; surface the bug in the log pane
                self.logs.add("sim", "ERROR", f"sim tick failed: {exc!r}")
            time.sleep(max(0.0, TICK - (time.monotonic() - t0)))

    # ------------------------------------------------------------------ RobotIO
    def publish_stop(self) -> bool:
        with self._lock:
            self.logs.add(BRIDGE, "WARN", "Host STOP received; holding position")
            if not self.bridge_up:
                return False
            if self.cmd and self.cmd.result is None:
                self.cmd.result = "ERROR HOST_STOP"
                self._ack(f"ERROR {self.cmd.id} HOST_STOP")
            for k in self.j:
                self.tgt[k] = self.j[k]
            return True

    def call_cancel(self) -> Optional[bool]:
        with self._lock:
            if not self.exec["busy"]:
                return True
            self._finish("cancelled", self._cur_step_name(), "operator cancel")
            return True

    def send_command(self, command: str, token: str, args: Optional[Dict[str, Any]] = None) -> None:
        with self._lock:
            if not self.supervisor.is_active("hp-moveit"):
                return  # nobody is listening; the console will observe that nothing started
            if self.exec["busy"]:
                self.exec["last_rejected"] = {"token": token, "reason": f"busy with {self.exec['command']}"}
                self.logs.add(EXECUTOR, "WARN", f"Busy — ignoring gesture {command}")
                return
            args = args or {}
            if command in RECOVERY:
                built = self._recovery_steps(command, args)
                if isinstance(built, str):
                    self.exec["last_rejected"] = {"token": token, "reason": built}
                    self.logs.add(EXECUTOR, "WARN", f"Rejected {command}: {built}")
                    return
                kind, steps, task = "sequence", built, None
            else:
                kind, steps, task = COMMANDS[command]
            now = self.clock()
            self.exec = self._idle_exec()
            self.exec.update(busy=True, command=command, request_token=token, t_start=time.time(),
                             steps=[st if isinstance(st, ExecStep) else ExecStep(st[0], st[1], st[2], dict(st[3]))
                                    for st in steps], idx=0,
                             is_sequence=kind == "sequence", args=args)
            if command in ("pick_up_coffee", "press_square_button"):
                self.undo_stack.clear()
                self._last_sequence = {"command": command}
            self._approval_response = None
            if task:
                self.task_id = task
            self.logs.add(EXECUTOR, "INFO", f"Starting sequence: {command}" if kind == "sequence"
                          else f"Executing {command}")
            self._exec_phase_t0 = now

    def send_approval(self, plan_id: Optional[int], approve: bool, operator: str) -> None:
        with self._lock:
            ap = self.exec.get("approval")
            if not ap or (plan_id is not None and plan_id != ap["plan_id"]):
                self.logs.add(EXECUTOR, "WARN", f"Ignoring approval for stale plan {plan_id}")
                return
            self._approval_response = approve

    def clear_octomap(self) -> bool:
        self.logs.add("move_group", "INFO", "Octomap cleared (/clear_octomap)")
        return self.supervisor.is_active("hp-moveit")

    def set_task_id(self, task_id: int) -> None:
        self.task_id = task_id

    def zero(self) -> Optional[bool]:
        """§7.10 ZERO: odometry and held base target := current pose. None when unsupported."""
        with self._lock:
            if not self.faults.zero_supported:
                return None
            if not self.bridge_up or (self.cmd and self.cmd.result is None) or self.exec["busy"]:
                self._ack("ERROR 0 BUSY")
                return False
            for k in BASE:
                self.j[k] = 0.0
                self.tgt[k] = 0.0
            self.undo_stack.clear()
            self._ack(f"ZEROED {self.boot_id}")
            self.logs.add(BRIDGE, "INFO", "ZEROED: odometry and held base target set to the current pose")
            return True

    def request_reset(self) -> bool:
        """Reset-based re-zero fallback: DTR toggle (laptop) or restart mac_serial_server.sh (Mac)."""
        with self._lock:
            if not self.supervisor.is_active("hp-bridge"):
                return False
            how = "restarting mac_serial_server.sh (DTR)" if self.serial_bridge else "toggling DTR on /dev/ttyACM0"
            self.logs.add("console", "WARN", f"Arduino reset requested: {how}")
            self._ack("ERROR 0 FIRMWARE_RESTART")
            self._open_session(reset=True)
            return True

    # ------------------------------------------------------------------ faults
    def set_fault(self, name: str, value: Any) -> Dict:
        with self._lock:
            if name == "push_base":
                dx = float(value)
                self.j["base_x"] += dx
                self.logs.add("sim", "INFO", f"robot pushed {dx * 100:+.0f} cm by hand")
                return {"ok": True}
            if name == "reset_arduino":
                if self.bridge_up:
                    self._ack("ERROR 0 FIRMWARE_RESTART")
                    self.logs.add(BRIDGE, "ERROR", "Arduino execution error: ERROR 0 FIRMWARE_RESTART")
                    self._open_session(reset=True)
                return {"ok": True}
            if name == "abort_scenario":
                # A realistic abort: the camera died, the detector sat on stale frames, refine timed out.
                self.faults.detector_scenario = "stale_frames"
                return {"ok": True}
            if name == "fail_unit":
                self.supervisor.fail(str(value))
                return {"ok": True}
            if not hasattr(self.faults, name):
                return {"ok": False, "error": f"unknown fault {name}"}
            setattr(self.faults, name, value)
            if name == "detector_scenario":
                self._cycle_n = 0
            self.logs.add("sim", "INFO", f"fault {name} = {value}")
            return {"ok": True}

    def fault_state(self) -> Dict:
        return dict(self.faults.__dict__)

    # ------------------------------------------------------------------ tick
    def tick(self, now: float) -> None:
        wall = time.time()
        units = self.supervisor.status()
        active = {u: s["active_state"] == "active" for u, s in units.items()}
        self._physics(now)
        self._bridge(now, active["hp-bridge"])
        self._executor(now, wall, active["hp-moveit"])
        self._cameras(now, wall, active["hp-camera-rx"])
        self._detector(now, wall, active["hp-vision"])
        if active["hp-tf"] and now - self._last_tf >= 0.1:
            self._last_tf = now
            self.store.update("tf/world_base", {"x": self.j["base_x"], "y": self.j["base_y"]}, "tf2 world→base_link")
        if self.faults.gpu_up and now - self._last_gpu >= 5.0:
            self._last_gpu = now
            self.store.update("gpu/health", {"status": 200, "latency_s": round(random.uniform(0.25, 0.5), 2)},
                              "GET 192.168.64.1:8765/health")

    def _physics(self, now: float) -> None:
        powered = self.faults.motor_power and self.bridge_up
        for k in self.j:
            prev = self.j[k]
            if not self.faults.motor_power and k in FOLDED:
                # unpowered arm sags under gravity until it rests on its folded limit
                err = FOLDED[k] - prev
                self.j[k] = prev + max(-12.0 * TICK, min(12.0 * TICK, err))
            elif powered:
                rate = 0.15 if k in ("base_x", "base_y") else (6.0 if k == "base_theta" else
                                                               (60.0 if k == "joint_grip_L" else 40.0))
                err = self.tgt[k] - prev
                step = max(-rate * TICK, min(rate * TICK, err))
                self.j[k] = prev + step
            self.vel[k] = (self.j[k] - prev) / TICK

    def _arm_still(self) -> bool:
        return all(abs(self.vel[k]) < 0.5 for k in ARM) and all(abs(self.vel[k]) < 0.005 for k in BASE)

    # -- bridge ----------------------------------------------------------
    def _open_session(self, reset: bool) -> None:
        now = self.clock()
        self.session_id = uuid.uuid4().hex[:8]
        self.session_start = now
        self.cmd = None
        self.bridge_up = True
        self._last_status = None
        self.logs.add(BRIDGE, "INFO", f"Connected {self.port} at 115200; waiting for firmware state feedback")
        if reset or self.boot_id is None:
            self._boot()
        self.logs.add(BRIDGE, "INFO", "Firmware STATE4 feedback OK (protocol STATE4)")
        self.store.push_event("/arduino/bridge_info", {"session_id": self.session_id, "boot_id": self.boot_id,
                                                        "port": self.port, "opened_wall": time.time(),
                                                        "protocol": "STATE4", "zero_supported": self.faults.zero_supported},
                              "/arduino/bridge_info")

    def _boot(self) -> None:
        """Firmware setup(): odometry zero wherever the robot is, base holds that spot, arm commanded upright."""
        self.boot_id = uuid.uuid4().hex[:6]
        self.undo_stack.clear()
        self.logs.add(BRIDGE, "WARN", "READY4 received: firmware booted (odometry reset)")
        for k in BASE:
            self.j[k] = 0.0
            self.tgt[k] = 0.0
        self.tgt["joint_elbow"] = 0.0
        self.tgt["joint_shoulder"] = 0.0
        self.tgt["joint_wrist"] = self.j["joint_wrist"]
        self.tgt["joint_grip_L"] = self.j["joint_grip_L"]

    def _close_session(self, reason: str) -> None:
        if self.bridge_up:
            self.bridge_up = False
            if self.cmd and self.cmd.result is None:
                self.cmd.result = "ERROR STALE_FEEDBACK"
                self._ack(f"ERROR {self.cmd.id} STALE_FEEDBACK")
            self.store.update("/arduino/ready", False, "/arduino/ready")
            self.logs.add(BRIDGE, "ERROR", reason)

    def _bridge(self, now: float, unit_active: bool) -> None:
        connected = self.faults.usb_connected
        if unit_active and not self._was_bridge_active:
            if connected:
                # serial bridge: the ROS bridge reopens the pty only, no DTR, no reset
                self._open_session(reset=not self.serial_bridge)
        if not unit_active and self._was_bridge_active:
            self._close_session("Bridge shutting down")
        self._was_bridge_active = unit_active
        if not unit_active:
            return
        if not connected:
            self._close_session("Arduino serial failure: [Errno 5] Input/output error")
            if now - self._last_retry_log >= 2.0:
                self._last_retry_log = now
                self.logs.add(BRIDGE, "WARN", f"Arduino serial port is not connected; retrying. ({self.port}"
                              + (": is vm_serial_client.sh / mac_serial_server.sh running?)" if self.serial_bridge else ")"))
            return
        if connected and not self.bridge_up:
            # _read_loop reconnects every 2 s. Direct USB: reopening resets the Mega. Serial bridge: the Mac
            # side reopens the USB device for the new client, which probably raises DTR once (bench test, §9).
            self._open_session(reset=True)
        # joint states at the STATE4 cadence (25 Hz)
        if int(now / 0.04) != int((now - TICK) / 0.04):
            self.store.update("/hardware/joint_states", dict(self.j), "/hardware/joint_states")
            self.store.update("/hardware/joint_states_si", self._si(self.j), "/hardware/joint_states")
        if now - self._last_ready_pub >= 5.0 or self._last_ready_pub < 0:
            self._last_ready_pub = now
            self.store.update("/arduino/ready", True, "/arduino/ready")
        still = self._arm_still()
        if still != self._last_status:
            self._last_status = still
            self.store.update("/arduino/status", still, "/arduino/status")
        self._arduino_cmd(now)

    def _ack(self, text: str) -> None:
        self.store.push_event("/arduino/ack", text, "/arduino/ack")

    def queue_arduino(self, targets: Dict[str, float], kind: str) -> int:
        cid = self._next_cmd_id
        self._next_cmd_id += 1
        self.cmd = ArduinoCmd(cid, dict(targets), self.clock(), kind)
        for k, v in targets.items():
            self.tgt[k] = v
        self._ack(f"ACK {cid}")
        return cid

    def _arduino_cmd(self, now: float) -> None:
        c = self.cmd
        if not c or c.result is not None:
            return
        done = all(abs(self.j[k] - v) < (0.01 if k in ("base_x", "base_y") else 1.0) for k, v in c.targets.items())
        if done and self._arm_still():
            c.result = "DONE"
            self._ack(f"DONE {c.id}")
            return
        elapsed = now - c.t_start
        if not self.faults.motor_power:
            base_err = abs(self.j["base_x"] - c.targets.get("base_x", self.j["base_x"]))
            if base_err > 0.01 and elapsed > 2.0:
                c.result = f"ERROR TRACKING_ERROR dx_cm={base_err * 100:.1f}"
                self._ack(f"ERROR {c.id} TRACKING_ERROR dx_cm={base_err * 100:.1f}")
            elif elapsed > 4.0:
                c.result = "ERROR SETTLE_TIMEOUT"
                self._ack(f"ERROR {c.id} SETTLE_TIMEOUT")

    # -- executor --------------------------------------------------------
    def _idle_exec(self) -> Dict[str, Any]:
        prev = getattr(self, "exec", None) or {}
        return {"busy": False, "command": None, "request_token": prev.get("request_token"), "steps": [],
                "idx": 0, "phase": None, "approval": None, "last_result": prev.get("last_result"),
                "last_rejected": prev.get("last_rejected"), "is_sequence": False, "t_start": None,
                "target": None, "plan": None, "arduino_id": None}

    def _cur_step_name(self) -> Optional[str]:
        e = self.exec
        if e["busy"] and e["steps"] and e["idx"] < len(e["steps"]):
            return e["steps"][e["idx"]].name
        return None

    def _finish(self, outcome: str, step: Optional[str], reason: str) -> None:
        e = self.exec
        wall = time.time()
        if e["steps"] and e["idx"] < len(e["steps"]):
            cur = e["steps"][e["idx"]]
            cur.status = "failed" if outcome in ("aborted", "failed") else ("cancelled" if outcome == "cancelled" else cur.status)
            cur.t_end = wall
            cur.note = reason if outcome != "succeeded" else cur.note
            for s in e["steps"][e["idx"] + 1:]:
                s.status = "skipped" if outcome != "succeeded" else s.status
        e["last_result"] = {"outcome": outcome, "command": e["command"], "step": step, "reason": reason, "t": wall}
        if outcome == "aborted":
            self.logs.add(EXECUTOR, "ERROR", f"Sequence {e['command']} ABORTED at {step}: {reason}")
        elif outcome == "cancelled":
            self.logs.add(EXECUTOR, "WARN", f"Sequence {e['command']} cancelled at {step}: {reason}")
        elif outcome == "succeeded":
            self.logs.add(EXECUTOR, "INFO", f"Sequence {e['command']} complete")
        if self.cmd and self.cmd.result is None and outcome != "succeeded":
            self.publish_stop()
        last_steps = e["steps"]
        self.exec = self._idle_exec()
        self.exec["steps"] = last_steps
        self.exec["finished_command"] = e["command"]

    def _executor(self, now: float, wall: float, moveit_active: bool) -> None:
        if not moveit_active:
            if self._was_moveit_active and self.exec["busy"]:
                self.exec = self._idle_exec()  # process died: no state is published, outcome unknown
            self._was_moveit_active = False
            return
        if not self._was_moveit_active:
            self.store.update("moveit/ready", True, "/move_action")
        self._was_moveit_active = True
        if now - self._last_moveit >= 2.0:
            self._last_moveit = now
            self.store.update("moveit/ready", True, "/move_action")
        e = self.exec
        if e["busy"]:
            self._exec_step(now, wall)
        if now - self._last_exec_pub >= 0.5:
            self._last_exec_pub = now
            self._exec_seq += 1
            self.store.update("/gesture_control/executor_state", self._exec_json(now), "/gesture_control/executor_state")

    def _exec_json(self, now: float) -> Dict[str, Any]:
        e = self.exec
        steps = [{"name": s.name, "type": s.type, "status": s.status, "t_start": s.t_start, "t_end": s.t_end,
                  "note": s.note, "requires_approval": s.requires_approval} for s in e["steps"]]
        seq = None
        if e["busy"] and e["steps"]:
            s = e["steps"][e["idx"]]
            seq = {"name": e["command"], "step_index": e["idx"] + 1, "step_count": len(e["steps"]),
                   "step_name": s.name, "step_type": s.type, "phase": e["phase"]}
        ap = None
        if e.get("approval"):
            ap = {k: v for k, v in e["approval"].items() if not k.startswith("_")}
            ap["remaining_sec"] = round(max(0.0, e["approval"]["_deadline_mono"] - now), 2)
        return {"seq": self._exec_seq, "busy": e["busy"], "command": e["command"] or e.get("finished_command"),
                "request_token": e["request_token"], "sequence": seq, "steps": steps, "approval": ap,
                "last_result": e["last_result"], "last_rejected": e["last_rejected"],
                "hardware_ready": self.bridge_up, "holding_cup": self.holding_cup,
                "undo_stack": list(reversed(self.undo_stack)),
                "capabilities": {"undo": True, "resume": True, "return_to_start": True},
                "resumable": self._resumable()}

    def _resumable(self) -> Optional[Dict[str, Any]]:
        lr = self.exec.get("last_result") or {}
        last = self._last_sequence
        if self.exec["busy"] or not last or lr.get("outcome") not in ("aborted", "failed", "cancelled"):
            return None
        names = [n for (n, *_r) in COMMANDS[last["command"]][1]]
        if lr.get("step") not in names:
            return None
        return {"command": last["command"], "step_index": names.index(lr["step"]) + 1, "step": lr["step"],
                "step_count": len(names)}

    def _abort(self, reason: str) -> None:
        self._finish("aborted", self._cur_step_name(), reason)

    def _exec_step(self, now: float, wall: float) -> None:
        e = self.exec
        if e["idx"] >= len(e["steps"]):
            self._finish("succeeded", None, "")
            return
        s: ExecStep = e["steps"][e["idx"]]
        if s.status == "pending":
            s.status, s.t_start = "running", wall
            self._exec_phase_t0 = now
            if s.type == "noop":
                s.status, s.t_end = "skipped", wall
                e["idx"] += 1
                if e["idx"] >= len(e["steps"]):
                    self._finish("succeeded", None, "")
                return
            e["phase"] = "detecting" if s.type == "detect" else (
                "gripper" if s.type == "gripper" and not s.requires_approval else "planning")
            e["plan"] = None
            e["arduino_id"] = None
            self.detections = []
            if e["is_sequence"]:
                self.logs.add(EXECUTOR, "INFO", f"[{e['command']}] step {e['idx'] + 1}/{len(e['steps'])}: {s.name}")
            return
        t = now - self._exec_phase_t0
        phase = e["phase"]
        if phase == "detecting":
            good = [d for d in self.detections if d["stage"] == "done"]
            if good:
                e["target"] = good[-1]["pose"]
                s.note = f"{len(good)} sample{'s' if len(good) != 1 else ''}"
                self._advance(now, wall)
            elif t > 30.0:
                self._abort(f"no stable detection on /molmopoint/task_{self.task_id}/pose within 30.0s")
            return
        if phase == "planning":
            if t < 0.8:
                return
            target = e.get("target")
            if s.type == "pose" and target and not (0.3 < target[2] < 1.4):
                self.logs.add(EXECUTOR, "WARN", f"IK preflight {s.name}: 0 valid, rejected=24")
                self._abort(f"MoveIt failed for {s.name}: no valid IK goal")
                return
            ends = self._resolve_targets(s.targets)
            plan_id = self._next_plan_id
            self._next_plan_id += 1
            e["plan"] = {"plan_id": plan_id, "targets": ends, "undo": self._undo_method(s) if s.undo_of else None}
            if e["plan"]["undo"]:
                u = e["plan"]["undo"]
                s.note = "reverse replay" if u["method"] == "reverse_replay" else "fallback: fresh plan"
                self.logs.add(EXECUTOR, "INFO" if u["method"] == "reverse_replay" else "WARN",
                              f"Undo {s.undo_of['step']}: {u['method']} ({u['reason']})")
            self.logs.add(EXECUTOR, "INFO", f"IK preflight {s.name}: 18 valid, rejected=6")
            self.store.update("/gesture_control/plan_preview", self._preview(plan_id, s, ends), "/gesture_control/plan_preview")
            if s.requires_approval:
                e["phase"] = "awaiting_approval"
                e["approval"] = {"plan_id": plan_id, "step": s.name, "required": True, "timeout_sec": 60.0,
                                 "deadline_wall": wall + 60.0, "_deadline_mono": now + 60.0}
                self._approval_response = None
                self.logs.add(EXECUTOR, "INFO", f"Plan for {s.name} is waiting for approval")
            else:
                s.auto_approved = True
                e["phase"] = "executing"
            self._exec_phase_t0 = now
            return
        if phase == "awaiting_approval":
            ap = e["approval"]
            if self._approval_response is True:
                e["approval"] = None
                e["phase"] = "executing"
                self._exec_phase_t0 = now
                self.logs.add(EXECUTOR, "INFO", f"Plan {ap['plan_id']} approved")
            elif self._approval_response is False:
                e["approval"] = None
                self._abort("plan rejected by operator")
            elif now >= ap["_deadline_mono"]:
                e["approval"] = None
                self._abort("approval timed out after 60.0 s")
            return
        if phase in ("executing", "gripper"):
            if not self.bridge_up:
                if t > 1.0:
                    self._abort("Arduino not ready (hardware joint state stale)")
                return
            if e["arduino_id"] is None:
                s.start_joints = dict(self.j)
                ends = e["plan"]["targets"] if e.get("plan") else self._resolve_targets(s.targets)
                e["arduino_id"] = self.queue_arduino(ends, "gripper" if s.type == "gripper" else "both")
                return
            c = self.cmd
            if c is None or c.id != e["arduino_id"]:
                self._abort("Arduino command lost")
                return
            if c.result == "DONE":
                e["phase"] = "verifying"
                self._exec_phase_t0 = now
            elif c.result:
                self.logs.add(EXECUTOR, "ERROR", f"Arduino execution error: {c.result.replace('ERROR', 'ERROR ' + str(c.id), 1)}")
                self._abort(f"Arduino execution error: {c.result.replace('ERROR', 'ERROR ' + str(c.id), 1)}")
            elif t > 20.0:
                self._abort("Arduino DONE not received within 20.0 s")
            return
        if phase == "verifying":
            if t >= 0.4:
                s.note = s.note or ("verified" if s.type != "gripper" else f"{now - self._exec_phase_t0 + 0.5:.1f} s")
                self.logs.add(EXECUTOR, "INFO", f"{s.name}: measured endpoint verified")
                self._advance(now, wall)

    def _advance(self, now: float, wall: float) -> None:
        e = self.exec
        s = e["steps"][e["idx"]]
        s.status, s.t_end = "done", wall
        if s.type == "gripper" and not s.note:
            s.note = f"{wall - s.t_start:.1f} s"
        self._record_undo(s, e["command"], wall)
        e["idx"] += 1
        if e["idx"] >= len(e["steps"]):
            if e["command"] == "return_to_start":
                self.undo_stack.clear()  # back at the origin: the recorded path no longer applies
            self._finish("succeeded", None, "")

    # -- undo / recovery (§5.4, §7.8) -------------------------------------
    def _record_undo(self, s: ExecStep, command: str, wall: float) -> None:
        grip = s.targets.get("joint_grip_L")
        if s.name == "close_gripper_at_target":
            self.holding_cup = True     # executor attaches detected_cup (attach_cup: true)
            self.logs.add(EXECUTOR, "INFO", "Attached detected_cup to grasp_tcp")
        elif s.type == "gripper" and grip is not None and grip <= 1.0 and self.holding_cup and \
                (command not in ("pick_up_coffee", "press_square_button")):
            self.holding_cup = False    # single-command gripper open detaches the cup
            self.logs.add(EXECUTOR, "INFO", "Detached detected_cup")
        if s.undo_of is not None:
            self.undo_stack = [r for r in self.undo_stack if r["undo_id"] != s.undo_of["undo_id"]]
            if s.undo_of["step"] == "close_gripper_at_target":
                self.holding_cup = False
            return
        if command in RECOVERY or s.type not in ("detect", "pose", "joint_state", "gripper"):
            return
        self._undo_seq += 1
        start = s.start_joints or dict(self.j)
        self.undo_stack.append({
            "undo_id": self._undo_seq, "step": s.name, "type": s.type, "command": command,
            "traj_file": None if s.type in ("detect", "gripper") else
            f"trajectory_logs/traj_{s.name}_{time.strftime('%Y%m%d_%H%M%S', time.localtime(s.t_start or wall))}.csv",
            "start_joints": {k: round(v, 4) for k, v in start.items()},
            "end_joints": {k: round(v, 4) for k, v in self.j.items()},
            "gripper_before": round(start["joint_grip_L"], 1), "t_done": wall,
            "endpoint_verified": s.type in ("pose", "joint_state"), "reversible": s.type != "detect"})

    def _undo_method(self, s: ExecStep) -> Dict[str, Any]:
        """Reverse replay when the robot is still where the recorded step ended; else plan fresh."""
        r = s.undo_of
        if s.type == "gripper":
            return {"method": "restore_gripper", "reason": f"gripper back to {r['gripper_before']:.0f}°",
                    "traj_file": None, "of_step": r["step"]}
        end = r["end_joints"]
        base = math.hypot(self.j["base_x"] - end["base_x"], self.j["base_y"] - end["base_y"])
        arm = max(abs(self.j[k] - end[k]) for k in ("joint_shoulder", "joint_elbow", "joint_wrist"))
        if base > 0.01 or arm > 2.3:
            why = f"base drifted {base * 100:.1f} cm" if base > 0.01 else f"arm drifted {arm:.1f}°"
            return {"method": "fallback_plan", "reason": f"{why} from the recorded endpoint; planning a fresh "
                    "joint-space move to the pre-step state", "traj_file": r["traj_file"], "of_step": r["step"]}
        return {"method": "reverse_replay", "reason": "state within tolerance of the recorded endpoint; "
                "reversed path re-checked against the current planning scene",
                "traj_file": r["traj_file"], "of_step": r["step"]}

    def _recovery_steps(self, command: str, args: Dict[str, Any]):
        if command == "return_to_start":
            steps = []
            if self.holding_cup and args.get("release_cup", True):
                steps.append(ExecStep("release_cup", "gripper", False, {"joint_grip_L": 0.0}))
            steps.append(ExecStep("home", "joint_state", True, dict(HOME_TARGETS)))
            return steps
        if command in ("undo_step", "undo_sequence"):
            stack = list(reversed(self.undo_stack))  # newest first
            if command == "undo_step":
                stack = [r for r in stack if r["reversible"]][:1]
            if not any(r["reversible"] for r in stack):
                return "nothing to undo"
            steps = []
            for r in stack:
                if not r["reversible"]:
                    steps.append(ExecStep(f"undo_{r['step']}", "noop", False, {}, note="detect: nothing to undo"))
                    continue
                if r["type"] == "gripper":
                    st = ExecStep(f"undo_{r['step']}", "gripper", True, {"joint_grip_L": r["gripper_before"]})
                else:
                    tg = {k: r["start_joints"][k] for k in ("base_x", "base_y", "base_theta", "joint_shoulder",
                                                             "joint_elbow", "joint_wrist")}
                    st = ExecStep(f"undo_{r['step']}", "undo", True, tg)
                st.undo_of = r
                steps.append(st)
            return steps
        if command == "resume":
            lr = self.exec.get("last_result") or {}
            last = self._last_sequence
            if not last or lr.get("outcome") not in ("aborted", "failed", "cancelled") or not lr.get("step"):
                return "no aborted sequence to resume"
            seq = COMMANDS[last["command"]][1]
            names = [n for (n, *_rest) in seq]
            k = int(args.get("step_index") or (names.index(lr["step"]) + 1 if lr["step"] in names else 0))
            if k < 1:
                return f"step {lr['step']} not in {last['command']}"
            steps = [ExecStep(n, t, a, dict(tg)) for (n, t, a, tg) in seq[k - 1:]]
            if steps[0].type != "detect":
                steps.insert(0, ExecStep("re_detect", "detect", False, {}))
            return steps
        return f"unknown recovery command {command}"

    def _resolve_targets(self, spec: Dict[str, float]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for k, v in spec.items():
            if k == "base_dx":
                out["base_x"] = self.j["base_x"] + v
            elif k == "base_dy":
                out["base_y"] = self.j["base_y"] + v
            elif k == "base_dtheta":
                out["base_theta"] = self.j["base_theta"] + v
            else:
                out[k] = v
        return out

    def _tcp(self, j: Dict[str, float]) -> List[float]:
        sh, el = math.radians(j["joint_shoulder"]), math.radians(j["joint_elbow"])
        reach = 0.28 * math.sin(sh) + 0.22 * math.sin(el)
        return [round(j["base_x"] + 0.03 + reach, 3), round(j["base_y"] + 0.02, 3),
                round(0.55 + 0.18 * math.sin(el - sh) + 0.04, 3)]

    def _preview(self, plan_id: int, s: ExecStep, ends: Dict[str, float]) -> Dict[str, Any]:
        start = dict(self.j)
        end = dict(start)
        end.update(ends)
        names = [k for k in ("joint_shoulder", "joint_elbow", "joint_wrist", "joint_grip_L") if k in ends]
        dur = max([abs(end[k] - start[k]) / 40.0 for k in names] +
                  [abs(end["base_x"] - start["base_x"]) / 0.15, abs(end["base_y"] - start["base_y"]) / 0.15, 0.6])
        tcp_end = self._tcp(end)
        tgt = self.exec.get("target")
        dist = math.dist(tcp_end, tgt) if tgt else None
        if s.name == "align_refined_grasp" and tgt:
            dist = 0.011
        plan = self.exec.get("plan") or {}
        return {"plan_id": plan_id, "step": s.name, "requires_approval": s.requires_approval,
                "command": self.exec.get("command"), "undo": plan.get("undo"),
                "base_start": {"x": round(start["base_x"], 3), "y": round(start["base_y"], 3), "theta": round(start["base_theta"], 1)},
                "base_end": {"x": round(end["base_x"], 3), "y": round(end["base_y"], 3), "theta": round(end["base_theta"], 1)},
                "joints": [{"name": k, "start": round(start[k], 1), "end": round(end[k], 1),
                            "max_excursion": round(abs(end[k] - start[k]), 1)} for k in names],
                "base_delta": {"dx": round(end["base_x"] - start["base_x"], 3), "dy": round(end["base_y"] - start["base_y"], 3),
                               "dtheta_deg": round(end["base_theta"] - start["base_theta"], 1)},
                "duration_sec": round(dur, 1), "waypoints": int(10 + dur * 9),
                "trajectory": self._downsample(start, end, dur),
                "tcp_end": tcp_end, "target": [round(v, 3) for v in tgt] if tgt else None,
                "target_distance_m": round(dist, 3) if dist is not None else None,
                "target_age_sec": 0.8 if tgt else None}

    @staticmethod
    def _si(j: Dict[str, float]) -> Dict[str, float]:
        """Console joint units (m, deg) → URDF units (m, rad), as in the executor's raw_<joint> CSV columns."""
        out = {}
        for k, v in j.items():
            out[k] = v if k in ("base_x", "base_y") else math.radians(v)
        out["joint_grip_R"] = -out.get("joint_grip_L", 0.0)
        return out

    def _downsample(self, start: Dict[str, float], end: Dict[str, float], dur: float, n: int = 40):
        """≤ 60 points of the plan (§7.2), eased like a time-parameterised trajectory."""
        pts = []
        for i in range(n + 1):
            u = i / n
            e = u * u * (3 - 2 * u)
            pts.append({"t": round(u * dur, 3),
                        "joints": {k: round(v, 5) for k, v in
                                   self._si({k: start[k] + (end[k] - start[k]) * e for k in start}).items()}})
        return pts

    # -- cameras ---------------------------------------------------------
    def _cameras(self, now: float, wall: float, rx_active: bool) -> None:
        if now - self._last_frame >= 0.2:
            self._last_frame = now
            if rx_active and self.faults.sender_alive:
                if self.faults.sender_frozen and self._frozen_color is not None:
                    color, depth = self._frozen_color, self._frozen_depth
                else:
                    color = sim_render.render_color(self.j["base_x"], CUP[0], wall,
                                                    self.faults.detector_scenario == "two_targets")
                    depth = sim_render.render_depth(self.j["base_x"], CUP[0], wall)
                    self._frozen_color, self._frozen_depth = color, depth
                for feed, img in (("d405_color", color), ("d405_depth", depth)):
                    f = self.frames.put_image(feed, img, now)
                    self.store.update(f"img/{feed}", {"seq": f.seq}, f"/cameras/d405/{'color' if feed == 'd405_color' else 'aligned_depth_to_color'}/image_raw")
        if now - self._last_d455 >= 0.2 and self.faults.d455_alive:
            self._last_d455 = now
            img = sim_render.render_overview(self.j["base_x"], self.j["base_y"], self.j["base_theta"], CUP[0],
                                             self.j["joint_shoulder"] > -10, wall)
            f = self.frames.put_image("d455_color", img, now)
            self.store.update("img/d455_color", {"seq": f.seq}, "/cameras/d455/color/image_raw")

    # -- detector --------------------------------------------------------
    def _detector(self, now: float, wall: float, active: bool) -> None:
        if not active:
            return
        if now - self._last_task_poll >= 2.0:
            self._last_task_poll = now
            self.store.update("/molmopoint/task_id", self.task_id, "/molmopoint task_id (GetParameters)")
        if now - self._last_cycle < 2.0:
            return
        self._last_cycle = now
        self._cycle_n += 1
        still = self.store.get("/arduino/status")
        arm_still = bool(still and still.value) and self.bridge_up
        if arm_still and self._still_since is None:
            self._still_since = now
        if not arm_still:
            self._still_since = None
        color = self.store.get("img/d405_color")
        frame_age = color.age(now) if color else None
        scen = self.faults.detector_scenario
        diag: Dict[str, Any] = {"backend": "remote", "cycle_sec": round(random.uniform(1.6, 2.4), 2),
                                "robot_moving": not arm_still, "rgbd_skew_sec": 0.004,
                                "frame_age_sec": round(frame_age, 2) if frame_age is not None else None,
                                "task_id": self.task_id}
        pose = None
        if color is None:
            diag.update(stage="inputs", reason="waiting_for_color_or_camera_info", detail="")
        elif scen == "stale_frames" or frame_age > 1.5:
            diag.update(stage="publish_gate", reason="frame_stale" if frame_age > 1.5 else "frame_before_motion_done",
                        detail=f"frame {frame_age:.1f} s old")
            if frame_age > 6 and self._cycle_n % 5 == 0:
                # the garbage pose seen on the failure day when frames died
                pose = (0.91, 0.05, -6.79)
                diag.update(stage="done", reason="pose_published", detail="depth sample invalid")
        elif not arm_still:
            diag.update(stage="motion_gate", reason="robot_moving", detail="")
        elif scen == "none":
            diag.update(stage="geometry", reason="model_returned_no_points", detail="There are none.",
                        model_text="There are none.")
        else:
            x, y, z = CUP
            if scen == "two_targets" and self._cycle_n % 2 == 0:
                x, y, z = SECOND_OBJECT
            if scen == "garbage_z":
                z = -6.79
            if scen == "high_first" and self._still_since and now - self._still_since < 2.5:
                z += 0.20
            pose = (x + random.gauss(0, 0.003), y + random.gauss(0, 0.003), z + random.gauss(0, 0.002))
            diag.update(stage="done", reason="pose_published", detail="")
        self.store.push_event("/molmopoint/diagnostics", diag, "/molmopoint/diagnostics")
        if pose:
            self.store.push_event(f"/molmopoint/task_{self.task_id}/pose", {"x": pose[0], "y": pose[1], "z": pose[2]},
                              f"/molmopoint/task_{self.task_id}/pose")
            self.detections.append({"stage": "done", "pose": pose, "t": now})
        # debug image: the detector draws on the frame it used
        if self._frozen_color is not None:
            point = None
            if pose and diag["stage"] == "done":
                cx, cy, s = sim_render.cup_projection(self.j["base_x"], CUP[0])
                point = (cx + (pose[1] - CUP[1]) * -300 * s, cy)
            img = sim_render.render_overlay(self._frozen_color, diag["stage"], diag["reason"], point,
                                            self.j["base_x"], CUP[0], diag["stage"] == "done")
            f = self.frames.put_image("d405_overlay", img, now)
            self.store.update("img/d405_overlay", {"seq": f.seq}, "/molmopoint/debug_image")
