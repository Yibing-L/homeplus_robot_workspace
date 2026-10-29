"""rclpy layer: ROS topics → SignalStore, console actions → ROS (UI_PLAN §2, §7). Implements RobotIO.

Works against today's unmodified robot code in degraded mode, and upgrades itself per signal as the
§7 changes land:
  - /gesture_control/executor_state (§7.1) present → structured timeline; else derived from
    /gesture_control/executor_status text + gesture_execution_map.yaml step names.
  - /gesture_control/plan_preview (§7.2) present → previews; else built from /display_planned_path.
  - /gesture_control/approve_plan subscriber (§7.3) → scoped approval; else the Bool topic.
  - /gesture_control/cancel (§7.3) → cancel; else STOP only.
  - /arduino/bridge_info (§7.4) → boot/session identity and ZERO support; else inferred from bridge logs.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import yaml
from PIL import Image as PILImage
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .images import FrameStore
from .log_filter import LogBuffer, parse_status_text
from .signals import SignalStore
from .supervisor import SystemdSupervisor

try:  # optional: only needed for the degraded-mode plan preview and MoveIt readiness
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import DisplayTrajectory
    from rclpy.action import ActionClient
except ImportError:  # pragma: no cover
    MoveGroup = DisplayTrajectory = ActionClient = None

try:
    from homeplus_interfaces.msg import GestureCommand
except ImportError:  # pragma: no cover
    GestureCommand = None

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
ARM_JOINTS = ("joint_elbow", "joint_shoulder", "joint_wrist", "joint_hand", "joint_grip_L")
EXECUTOR_NODE = "gesture_motion_executor"
FEEDS = {  # feed id → (topic, max publish-to-browser rate)
    "d405_overlay": ("/molmopoint/debug_image", 2.0),
    "d405_color": ("/cameras/d405/color/image_raw", 5.0),
    "d405_depth": ("/cameras/d405/aligned_depth_to_color/image_raw", 5.0),
    "d455_color": ("/cameras/d455/color/image_raw", 5.0),
}
# gesture_behavior_map.yaml / gesture_execution_map.yaml
COMMAND_TYPES = {"home": ("joint_state", 0), "pick_up_coffee": ("sequence", -1), "press_square_button": ("sequence", -1),
                 "place_cup_on_drip_tray": ("sequence", -1),
                 "gripper_open": ("gripper", -1), "gripper_close": ("gripper", -1), "back_up": ("joint_state", 0)}
DEPTH_RANGE = (0.15, 1.50)


def image_to_pil(msg: Image) -> Optional[PILImage.Image]:
    """sensor_msgs/Image → PIL without cv_bridge. Depth is colourised over DEPTH_RANGE."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    h, w = msg.height, msg.width
    try:
        if enc in ("rgb8", "bgr8"):
            a = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
            return PILImage.fromarray(a[:, :, ::-1] if enc == "bgr8" else a)
        if enc in ("rgba8", "bgra8"):
            a = buf.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
            return PILImage.fromarray(a[:, :, ::-1] if enc == "bgra8" else a)
        if enc in ("mono8", "8uc1"):
            return PILImage.fromarray(buf.reshape(h, msg.step)[:, :w])
        if enc in ("16uc1", "mono16", "32fc1"):
            dt = np.float32 if enc == "32fc1" else np.uint16
            d = np.frombuffer(bytes(msg.data), dtype=dt).reshape(h, msg.step // np.dtype(dt).itemsize)[:, :w].astype(np.float32)
            if enc != "32fc1":
                d = d / 1000.0  # millimetres
            lo, hi = DEPTH_RANGE
            v = np.clip((d - lo) / (hi - lo), 0, 1)
            rgb = np.stack([np.clip(1.5 - np.abs(4 * v - k), 0, 1) for k in (3, 2, 1)], axis=-1)
            rgb[d <= 0] = 0
            return PILImage.fromarray((rgb * 255).astype(np.uint8))
    except ValueError:
        return None
    return None


class _Fallback:
    """Degraded mode (§4.3): an executor_state built from status text, busy Bool and the step map."""

    def __init__(self, step_map: Dict[str, List[Dict[str, Any]]]):
        self.step_map = step_map
        self.state: Dict[str, Any] = {"busy": False, "command": None, "request_token": None, "sequence": None,
                                      "steps": [], "approval": None, "last_result": None, "last_rejected": None,
                                      "holding_cup": None, "undo_stack": [],
                                      "capabilities": {"undo": False, "resume": False, "return_to_start": True},
                                      "derived_from": "status_text"}
        self._approval_deadline: Optional[float] = None
        self.plan_counter = 0

    def on_status(self, text: str, now: float, pending_token: Optional[str]) -> None:
        s = self.state
        wall = time.time()
        if text.startswith("Starting sequence:"):
            name = text.split(":", 1)[1].strip()
            s.update(busy=True, command=name, request_token=pending_token, approval=None,
                     steps=[{"name": st["name"], "type": st.get("type", ""), "status": "pending", "t_start": None, "t_end": None,
                             "note": "", "requires_approval": bool(st.get("require_trajectory_approval", True))}
                            for st in self.step_map.get(name, [])])
            return
        p = parse_status_text(text)
        if not p:
            return
        if p["kind"] == "holding":
            s["holding_cup"] = p["holding"]
            return
        if p["kind"] == "step":
            s["busy"] = True
            s["command"] = p["sequence"]
            names = [st["name"] for st in s["steps"]]
            if p["step_name"] not in names:  # map unknown or changed: grow the list as steps appear
                s["steps"].append({"name": p["step_name"], "type": "", "status": "pending", "t_start": None, "t_end": None,
                                   "note": "", "requires_approval": False})
            for st in s["steps"]:
                if st["status"] == "running":
                    st.update(status="done", t_end=wall)
            cur = next(st for st in s["steps"] if st["name"] == p["step_name"])
            cur.update(status="running", t_start=wall)
            s["sequence"] = {"name": p["sequence"], "step_index": p["step_index"], "step_count": p["step_count"],
                             "step_name": p["step_name"], "step_type": cur.get("type"), "phase": "executing"}
        elif p["kind"] == "awaiting_approval":
            self.plan_counter += 1
            self._approval_deadline = now + 60.0
            if s["sequence"]:
                s["sequence"]["phase"] = "awaiting_approval"
            else:
                # a single command (e.g. back_up) awaiting approval: show it as a one-step sequence so the
                # approval card appears; on_busy clears it when the executor goes idle.
                s["sequence"] = {"name": s.get("command") or p["step_name"], "step_index": 1, "step_count": 1,
                                 "step_name": p["step_name"], "step_type": "", "phase": "awaiting_approval"}
            s["approval"] = {"plan_id": None, "step": p["step_name"], "required": True, "timeout_sec": 60.0,
                             "deadline_wall": wall + 60.0, "remaining_sec": 60.0}
        elif p["kind"] in ("aborted", "failed"):
            for st in s["steps"]:
                if st["status"] == "running":
                    st.update(status="failed", t_end=wall, note=p["reason"])
                elif st["status"] == "pending":
                    st["status"] = "skipped"
            s.update(busy=False, sequence=None, approval=None,
                     last_result={"outcome": p["kind"], "command": p.get("sequence") or p.get("command"),
                                  "step": p.get("step"), "reason": p["reason"], "t": wall})
        if "approved" in text and s.get("approval"):
            s["approval"] = None
            if s["sequence"]:
                s["sequence"]["phase"] = "executing"

    def on_busy(self, busy: bool) -> None:
        s = self.state
        if s["busy"] and not busy:
            wall = time.time()
            for st in s["steps"]:
                if st["status"] == "running":
                    st.update(status="done", t_end=wall)
            if not s.get("last_result") or s["last_result"].get("t", 0) < wall - 2:
                s["last_result"] = {"outcome": "succeeded", "command": s["command"], "step": None, "reason": "", "t": wall}
            s.update(sequence=None, approval=None)
        s["busy"] = busy

    def snapshot(self, now: float) -> Dict[str, Any]:
        s = json.loads(json.dumps(self.state))
        if s.get("approval") and self._approval_deadline is not None:
            s["approval"]["remaining_sec"] = max(0.0, self._approval_deadline - now)
        return s


class RosIO:
    mode = "ros"

    def __init__(self, store: SignalStore, frames: FrameStore, logs: LogBuffer, profile: str = "mac_vm",
                 profile_path: Optional[Path] = None):
        self.store, self.frames, self.logs, self.profile = store, frames, logs, profile
        cfg_dir = Path(__file__).resolve().parent.parent / "config" / "profiles"
        prof = yaml.safe_load((profile_path or cfg_dir / f"{profile}.yaml").read_text())
        self.prof = prof
        self.supervisor = SystemdSupervisor(prof.get("units", {}), logs)
        self._fallback = _Fallback(self._load_step_map(prof))
        self._structured_at: Optional[float] = None
        self._pending_token: Optional[str] = None
        self._last_frame_put: Dict[str, float] = {}
        self._bridge_info: Dict[str, Any] = {}
        self._session = 0
        self._node: Optional[Node] = None
        self._graph: Dict[str, int] = {}
        self._external: Dict[str, List[str]] = {}

    @property
    def structured(self) -> bool:
        return self._structured_at is not None and time.monotonic() - self._structured_at < 5.0

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        n = self._node = Node("operator_console")
        cb = ReentrantCallbackGroup()
        sub = lambda typ, topic, fn, qos=10: n.create_subscription(typ, topic, fn, qos, callback_group=cb)  # noqa: E731
        sub(JointState, "/hardware/joint_states", self._on_joints, qos_profile_sensor_data)
        sub(Bool, "/arduino/ready", lambda m: self.store.update("/arduino/ready", m.data, "/arduino/ready"), LATCHED)
        sub(Bool, "/arduino/status", lambda m: self.store.update("/arduino/status", m.data, "/arduino/status"), LATCHED)
        sub(String, "/arduino/ack", lambda m: self.store.push_event("/arduino/ack", m.data, "/arduino/ack"), RELIABLE)
        sub(String, "/arduino/bridge_info", self._on_bridge_info, LATCHED)
        sub(String, "/gesture_control/executor_state", self._on_executor_state, LATCHED)
        sub(String, "/gesture_control/plan_preview", self._on_plan_preview, LATCHED)
        sub(String, "/gesture_control/executor_status", self._on_status, RELIABLE)
        sub(Bool, "/gesture_control/executor_busy", lambda m: self._fallback.on_busy(m.data), RELIABLE)
        sub(String, "/molmopoint/diagnostics", self._on_diag, RELIABLE)
        for task in (1, 2, 7):  # actions.TASK_COMMANDS
            sub(PoseStamped, f"/molmopoint/task_{task}/pose",
                lambda m, t=task: self.store.push_event(f"/molmopoint/task_{t}/pose",
                                                        {"x": m.pose.position.x, "y": m.pose.position.y, "z": m.pose.position.z},
                                                        f"/molmopoint/task_{t}/pose"))
        for fid, (topic, _rate) in FEEDS.items():
            sub(Image, topic, lambda m, f=fid, t=topic: self._on_image(f, t, m), qos_profile_sensor_data)
        # /rosout is transient-local in Humble: subscribe the same way so a node's first lines (logged before
        # discovery completes, e.g. the bridge's "Connected …") still arrive.
        sub(Log, "/rosout", self._on_rosout, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if DisplayTrajectory is not None:
            sub(DisplayTrajectory, "/display_planned_path", self._on_display_trajectory)
        self._stop_pub = n.create_publisher(Bool, "/arduino/stop", RELIABLE)
        self._zero_pub = n.create_publisher(Bool, "/arduino/zero", RELIABLE)
        self._approve_bool = n.create_publisher(Bool, "/gesture_control/approve_planned_path", RELIABLE)
        self._approve_scoped = n.create_publisher(String, "/gesture_control/approve_plan", RELIABLE)
        self._cmd_pub = n.create_publisher(GestureCommand, "/gesture_control/command", 10) if GestureCommand else None
        self._cancel_cli = n.create_client(Trigger, "/gesture_control/cancel", callback_group=cb)
        # The collision map is nvblox's mesh in the MoveIt scene; move_group no longer has
        # an OctoMap (/clear_octomap). The action keeps its "clear_octomap" id so the
        # built UI bundle keeps working.
        self._map_clear_cli = n.create_client(Trigger, "/nvblox/clear", callback_group=cb)
        self._getp = n.create_client(GetParameters, "/molmopoint/get_parameters", callback_group=cb)
        self._setp = n.create_client(SetParameters, "/molmopoint/set_parameters", callback_group=cb)
        self._move = ActionClient(n, MoveGroup, "/move_action", callback_group=cb) if MoveGroup else None
        if tf2_ros is not None:
            self._tf_buf = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buf, n)
        n.create_timer(0.1, self._tick_fast, callback_group=cb)
        n.create_timer(0.5, self._tick_executor, callback_group=cb)
        n.create_timer(2.0, self._tick_slow, callback_group=cb)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(n)
        threading.Thread(target=ex.spin, name="rclpy", daemon=True).start()
        self.supervisor.start_polling()
        threading.Thread(target=self._probe_loop, name="probe", daemon=True).start()
        url = self.prof.get("gpu_health_url")
        if url:
            threading.Thread(target=self._gpu_loop, args=(url,), name="gpu-health", daemon=True).start()

    @staticmethod
    def _load_step_map(prof: Dict[str, Any]) -> Dict[str, List[Dict[str, Any]]]:
        path = Path(os.path.expanduser(prof.get("execution_map", ""))) if prof.get("execution_map") else \
            Path(__file__).resolve().parents[2] / "homeplus_moveit_config" / "config" / "gesture_execution_map.yaml"
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except OSError:
            return {}
        cmds = data.get("commands", data)
        return {k: v.get("steps", []) for k, v in cmds.items() if isinstance(v, dict) and v.get("type") == "sequence"}

    # ------------------------------------------------------------------ callbacks
    def _on_joints(self, m: JointState) -> None:
        si = dict(zip(m.name, m.position))
        if "base_x" not in si:
            return
        console = {k: (v if k in ("base_x", "base_y") else math.degrees(v)) for k, v in si.items()}
        self.store.update("/hardware/joint_states", console, "/hardware/joint_states")
        self.store.update("/hardware/joint_states_si", si, "/hardware/joint_states")

    def _on_bridge_info(self, m: String) -> None:
        try:
            info = json.loads(m.data)
        except ValueError:
            return
        self._bridge_info = info
        self.store.push_event("/arduino/bridge_info", info, "/arduino/bridge_info")

    def _on_executor_state(self, m: String) -> None:
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self._structured_at = time.monotonic()
        self.store.update("/gesture_control/executor_state", st, "/gesture_control/executor_state")

    def _on_plan_preview(self, m: String) -> None:
        try:
            self.store.update("/gesture_control/plan_preview", json.loads(m.data), "/gesture_control/plan_preview")
        except ValueError:
            pass

    def _on_status(self, m: String) -> None:
        self.logs.add(EXECUTOR_NODE, "INFO", m.data)
        before = self._fallback.state.get("holding_cup")
        self._fallback.on_status(m.data, time.monotonic(), self._pending_token)
        holding = self._fallback.state.get("holding_cup")
        if holding is not None and holding != before:
            # the next task follows from what the gripper holds: look for the drip tray while
            # holding the cup (task 7), for the cup otherwise (task 1)
            self.set_task_id(7 if holding else 1)

    def _on_diag(self, m: String) -> None:
        try:
            self.store.push_event("/molmopoint/diagnostics", json.loads(m.data), "/molmopoint/diagnostics")
        except ValueError:
            pass

    def _on_image(self, fid: str, topic: str, m: Image) -> None:
        now = self.store.clock()
        if now - self._last_frame_put.get(fid, -1) < 1.0 / FEEDS[fid][1]:
            self.store.update(f"img/{fid}", {"seq": None}, topic)  # receipt still counts for rate and age
            return
        img = image_to_pil(m)
        if img is None:
            return
        if img.width > 640:
            img = img.resize((640, int(img.height * 640 / img.width)))
        self._last_frame_put[fid] = now
        f = self.frames.put_image(fid, img, now)
        self.store.update(f"img/{fid}", {"seq": f.seq}, topic)

    def _source_for(self, node: str) -> str:
        """Which process said this? ROS log lines carry only the node name, so say so when it's ambiguous."""
        n = self._graph.get(node, 0)
        if n > 1:
            return f"ambiguous: {n} nodes named {node}"
        unit = self.supervisor.node_unit(node)
        if unit is None:
            return "rosout"
        if self._external.get(unit):
            return "outside console" if not self.supervisor.is_active(unit) else f"ambiguous: {unit} + outside console"
        return f"unit:{unit}" if self.supervisor.is_active(unit) else "rosout"

    def _on_rosout(self, m: Log) -> None:
        name = m.name.split(".")[-1]
        if name == "operator_console":
            return
        if "acquisition-time TF still missing" in m.msg:
            self.store.push_event("tf/drop", name, "/rosout")
        line = self.logs.add(name, LEVEL.get(m.level, "INFO"), m.msg, source=self._source_for(name))
        if name == "arduino_bridge" and not self._bridge_info and (
                "waiting for firmware state feedback" in m.msg or "READY" in m.msg):
            # Without §7.4 a reopened port (= Arduino reset) is only visible in the log.
            self._session += 1
            self.store.push_event("/arduino/bridge_info", {"session_id": f"log{self._session}",
                                                            "boot_id": f"log{self._session}", "zero_supported": False,
                                                            "derived_from": line.text}, "rosout")

    def _on_display_trajectory(self, m: Any) -> None:
        """Degraded-mode preview (§7.2 fallback) from MoveIt's DisplayTrajectory."""
        if not m.trajectory:
            return
        jt = m.trajectory[0].joint_trajectory
        mdj = m.trajectory[0].multi_dof_joint_trajectory
        if not jt.points:
            return
        names = list(jt.joint_names)
        first, last = jt.points[0], jt.points[-1]
        start = dict(zip(names, first.positions))
        end = dict(zip(names, last.positions))
        dur = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        step = len(jt.points) / 40.0
        traj = [{"t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, "joints": dict(zip(names, p.positions))}
                for p in jt.points[:: max(1, int(step))]]
        self._fallback.plan_counter += 1
        ap = self._fallback.state.get("approval")
        preview = {"plan_id": None, "step": ap["step"] if ap else "plan", "requires_approval": bool(ap),
                   "command": self._fallback.state.get("command"), "undo": None,
                   "joints": [{"name": k, "start": round(math.degrees(start[k]), 1), "end": round(math.degrees(end[k]), 1),
                               "max_excursion": round(abs(math.degrees(end[k] - start[k])), 1)}
                              for k in names if k in ARM_JOINTS],
                   "base_delta": {"dx": round(end.get("base_x", 0) - start.get("base_x", 0), 3),
                                  "dy": round(end.get("base_y", 0) - start.get("base_y", 0), 3),
                                  "dtheta_deg": round(math.degrees(end.get("base_theta", 0) - start.get("base_theta", 0)), 1)},
                   "base_start": {"x": start.get("base_x", 0), "y": start.get("base_y", 0), "theta": math.degrees(start.get("base_theta", 0))},
                   "base_end": {"x": end.get("base_x", 0), "y": end.get("base_y", 0), "theta": math.degrees(end.get("base_theta", 0))},
                   "duration_sec": round(dur, 2), "waypoints": len(jt.points), "tcp_end": [0.0, 0.0, 0.0],
                   "target": None, "target_distance_m": None, "target_age_sec": None, "trajectory": traj,
                   "derived_from": "/display_planned_path", "multi_dof": bool(mdj.points)}
        self.store.update("/gesture_control/plan_preview", preview, "/display_planned_path")

    # ------------------------------------------------------------------ timers
    def _tick_fast(self) -> None:
        if tf2_ros is None:
            return
        try:
            t = self._tf_buf.lookup_transform("world", "base_link", rclpy.time.Time())
            self.store.update("tf/world_base", {"x": t.transform.translation.x, "y": t.transform.translation.y},
                              "tf2 world→base_link")
        except Exception:  # noqa: BLE001 (tf2 raises several types; any failure = not fresh)
            pass

    def _tick_executor(self) -> None:
        """Publish the degraded executor_state only while the executor node exists (liveness)."""
        if self.structured:
            return
        nodes = self._node.get_node_names()
        if EXECUTOR_NODE in nodes:
            st = self._fallback.snapshot(time.monotonic())
            st["seq"] = int(time.monotonic() * 2)
            self.store.update("/gesture_control/executor_state", st, "executor_status (degraded)")
        self.store.update("graph/foreign", self._foreign_publishers(), "graph")

    def _tick_slow(self) -> None:
        if self._move is not None and self._move.server_is_ready():
            self.store.update("moveit/ready", True, "/move_action")
        if self._getp.service_is_ready():
            req = GetParameters.Request(names=["task_id"])
            fut = self._getp.call_async(req)
            fut.add_done_callback(lambda f: f.result() and f.result().values and self.store.update(
                "/molmopoint/task_id", f.result().values[0].integer_value, "/molmopoint task_id (GetParameters)"))

    def _probe_loop(self) -> None:
        """Every 2 s: ROS graph node counts and, per unit, evidence of an instance outside the console."""
        while True:
            try:
                counts: Dict[str, int] = {}
                for name, _ns in self._node.get_node_names_and_namespaces():
                    counts[name] = counts.get(name, 0) + 1
                for name, c in counts.items():
                    if c > 1 and self._graph.get(name, 0) < 2:
                        # discovery lags the duplicate's first lines; those can't be attributed either
                        self.logs.retag(name, time.time() - 6.0, f"ambiguous: {c} nodes named {name}")
                self._graph = counts
                self.store.update("graph/nodes", counts, "ROS graph")
                ext = {u: self.supervisor.external(u, counts) for u in self.supervisor.probes}
                self._external = ext
                self.store.update("graph/external", ext, "/proc + ROS graph")
            except Exception as exc:  # noqa: BLE001 (a probe failure must not kill the loop)
                self.logs.add("console", "WARN", f"probe failed: {exc!r}")
            time.sleep(2.0)

    def external_instance(self, unit: str) -> List[str]:
        """Fresh check at action time (not the 2 s cache): is this component already running elsewhere?"""
        counts: Dict[str, int] = {}
        for name, _ns in self._node.get_node_names_and_namespaces():
            counts[name] = counts.get(name, 0) + 1
        return self.supervisor.external(unit, counts)

    def _gpu_loop(self, url: str) -> None:
        while True:
            t0 = time.monotonic()
            try:
                with urllib.request.urlopen(url, timeout=3) as r:
                    if r.status == 200:
                        self.store.update("gpu/health", {"status": 200, "latency_s": round(time.monotonic() - t0, 2)}, f"GET {url}")
            except OSError:
                pass
            time.sleep(5.0)

    def _foreign_publishers(self) -> List[str]:
        out = []
        for topic in ("/gesture_control/command", "/gesture_control/approve_planned_path", "/arduino/command_queue"):
            for info in self._node.get_publishers_info_by_topic(topic):
                # gesture_behavior is part of gesture_control.launch.py (gesture -> command).
                if info.node_name not in ("operator_console", EXECUTOR_NODE, "gesture_behavior"):
                    out.append(f"{info.node_name} → {topic}")
        return out

    # ------------------------------------------------------------------ RobotIO
    def publish_stop(self) -> bool:
        reached = self._stop_pub.get_subscription_count() > 0
        for _ in range(3):
            self._stop_pub.publish(Bool(data=True))
            time.sleep(0.02)
        return reached and self.store.get("/hardware/joint_states") is not None

    def call_cancel(self) -> Optional[bool]:
        if not self._cancel_cli.service_is_ready():
            return None
        self._cancel_cli.call_async(Trigger.Request())
        return True

    def send_command(self, command: str, token: str, args: Optional[Dict] = None) -> None:
        if command == "return_to_start" and not self.structured:
            threading.Thread(target=self._compose_return, args=(token, args or {}), daemon=True).start()
            return
        if command in ("undo_step", "undo_sequence", "resume") and not self.structured:
            st = self._fallback.state
            st["last_rejected"] = {"token": token, "reason": f"{command} needs the executor changes in §7.8/§7.11"}
            return
        self._publish_command(command, token)

    def _publish_command(self, command: str, token: str) -> None:
        if self._cmd_pub is None:
            self.logs.add("console", "ERROR", "homeplus_interfaces not importable: cannot send commands")
            return
        ctype, gid = COMMAND_TYPES.get(command, ("command", -1))
        msg = GestureCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = f"console:{token}"   # request token (§7.1); executors that echo it get verified starts
        msg.gesture_id = gid
        msg.gesture_label = f"console:{token}"
        msg.confidence = 1.0
        msg.command_name = command
        msg.command_type = ctype
        self._pending_token = token
        self._fallback.state["request_token"] = token
        self._cmd_pub.publish(msg)

    def _compose_return(self, token: str, args: Dict[str, Any]) -> None:
        """M1 Return to start without §7.9: optional gripper_open (detaches the cup), then home."""
        def idle() -> bool:
            s = self.store.get("/gesture_control/executor_state")
            return bool(s and s.value and not s.value.get("busy"))
        holding = (self.store.get("/gesture_control/executor_state").value or {}).get("holding_cup") \
            if self.store.get("/gesture_control/executor_state") else None
        if args.get("release_cup", True) and holding is not False:
            self._publish_command("gripper_open", token + "-g")
            t0 = time.monotonic()
            time.sleep(1.0)
            while not idle() and time.monotonic() - t0 < 30:
                time.sleep(0.2)
        self._publish_command("home", token)

    def send_approval(self, plan_id: Optional[int], approve: bool, operator: str) -> None:
        if plan_id is not None and self._approve_scoped.get_subscription_count() > 0:
            self._approve_scoped.publish(String(data=json.dumps({"plan_id": plan_id, "approve": approve, "operator": operator})))
        else:
            self._approve_bool.publish(Bool(data=approve))

    def clear_octomap(self) -> bool:
        if not self._map_clear_cli.service_is_ready():
            return False
        # nvblox answers only after MoveIt has dropped the mesh; fire and forget, the
        # mapper logs a rejected or timed-out reset itself.
        self._map_clear_cli.call_async(Trigger.Request())
        return True

    def set_task_id(self, task_id: int) -> None:
        if not self._setp.service_is_ready():
            return
        p = Parameter(name="task_id", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(task_id)))
        self._setp.call_async(SetParameters.Request(parameters=[p]))

    def zero(self) -> Optional[bool]:
        if not self._bridge_info.get("zero_supported"):
            return None
        if self._zero_pub.get_subscription_count() == 0:
            return False
        self._zero_pub.publish(Bool(data=True))
        return True

    def request_reset(self) -> bool:
        """Laptop: reopening the USB port toggles DTR → restart hp-bridge. Mac: needs mac_agent (M3)."""
        if self.prof.get("serial_bridge"):
            return False
        self.supervisor.restart("hp-bridge")
        return True
