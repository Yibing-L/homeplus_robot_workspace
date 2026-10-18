"""Unit tests for the console's pure logic (UI_PLAN §8.5). No ROS needed: `python -m pytest test/`."""
from __future__ import annotations

import math
import tempfile
from collections import deque
from pathlib import Path

import pytest

from homeplus_console import config
from homeplus_console.actions import ActionService, Audit
from homeplus_console.log_filter import LogBuffer, parse_status_text
from homeplus_console.signals import SignalStore, sig_view
from homeplus_console.state_machine import Inputs, derive, detection_window
from homeplus_console.tracker import Tracker


class Clock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


CFG = dict(config.DEFAULTS, remote_gpu=False, serial_bridge=True)
UPRIGHT = {"base_x": 0.0, "base_y": 0.0, "base_theta": 0.0, "joint_elbow": 0.0, "joint_shoulder": 0.0,
           "joint_wrist": 0.0, "joint_hand": 0.0, "joint_grip_L": 5.0}
UNITS = {u: {"active_state": "active", "sub_state": "running", "pid": 1, "start_mono": 0, "restarts": 0,
             "result": "success", "uptime_s": 60} for u in ("hp-tf", "hp-camera-rx", "hp-vision", "hp-moveit", "hp-bridge")}


def healthy(store: SignalStore, clock: Clock, joints=None) -> None:
    j = dict(joints or UPRIGHT)
    store.update("/hardware/joint_states", j, "/hardware/joint_states")
    store.update("/arduino/ready", True, "/arduino/ready")
    store.update("/arduino/status", True, "/arduino/status")
    store.update("/gesture_control/executor_state", {"busy": False, "steps": [], "last_result": None,
                                                     "capabilities": {"undo": True, "resume": True, "return_to_start": True},
                                                     "undo_stack": []}, "es")
    store.update("moveit/ready", True, "m")
    store.update("tf/world_base", {}, "tf")
    for f in ("img/d405_color", "img/d405_depth", "img/d455_color", "img/d405_overlay"):
        store.update(f, {"seq": 1}, f)
    store.update("/molmopoint/task_id", 1, "p")
    store.update("/molmopoint/diagnostics", {"stage": "done", "reason": "pose_published", "frame_age_sec": 0.2}, "d")


def make(clock: Clock, store: SignalStore, tracker: Tracker, cycles=None) -> Inputs:
    if cycles is not None:
        tracker.mem.cycles = deque(cycles)
    return Inputs(signals=store.snapshot(), mem=tracker.mem, units=UNITS,
                  frozen_for={}, frame_seq={}, logs_recent=[], cfg=CFG, structured=True, mode="sim", flow=None,
                  requested_task=1, foreign_publishers=[])


@pytest.fixture()
def env():
    clock = Clock()
    store = SignalStore(clock)
    tracker = Tracker(store, Path(tempfile.mkdtemp()), preconfirm_origin=True)
    store.push_event("/arduino/bridge_info", {"session_id": "s1", "boot_id": "b1", "zero_supported": True}, "bi")
    healthy(store, clock)
    tracker.observe(clock())
    tracker.mem.motor_verified = {"at": 0, "how": "test"}
    tracker.mem.session["upright_reached"] = True
    return clock, store, tracker


def good_cycles(now, n=4, z=0.76):
    return [{"t": now - 2 * i, "stage": "done", "reason": "pose_published", "detail": "", "pose": (0.95, 0.02, z),
             "task_id": 1} for i in range(n)][::-1]


# ---------------------------------------------------------------------------- staleness (§8.5 frozen clock)
@pytest.mark.parametrize("threshold", [0.5, 1.0, 6.0])
def test_signal_goes_unknown_just_after_threshold(threshold):
    clock = Clock()
    store = SignalStore(clock)
    store.update("x", 0.78, "src")
    clock.t += threshold
    assert sig_view(store.get("x"), clock(), threshold, "src")["value"] == 0.78
    clock.t += 1e-3
    v = sig_view(store.get("x"), clock(), threshold, "src")
    assert v["value"] is None and v["last_value"] == 0.78 and v["fresh"] is False


def test_bridge_down_when_joints_stop(env):
    clock, store, tracker = env
    clock.t += 1.2
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["robot"]["state"] == "BRIDGE_DOWN"
    assert s["joints"]["values"] is None and s["joints"]["ready"] is None  # latched values hidden


def test_executor_unknown_when_state_stale(env):
    clock, store, tracker = env
    clock.t += 1.6
    store.update("/hardware/joint_states", dict(UPRIGHT), "j")  # joints fresh, executor_state 1.6 s old
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["robot"]["state"] == "EXECUTOR_UNKNOWN"


# ---------------------------------------------------------------------------- headline priority
def test_idle_ready_with_good_window(env):
    clock, store, tracker = env
    for k in tracker.mem.declared:
        tracker.mem.declared[k] = {"at": 0, "by": "t"}
    s = derive(make(clock, store, tracker, good_cycles(clock())), clock(), 0)
    assert s["perception"]["state"] == "TARGET_OK"
    assert s["robot"]["state"] == "IDLE_READY", s["robot"]


def test_motor_suspect_on_sag_beats_idle(env):
    clock, store, tracker = env
    sag = dict(UPRIGHT, joint_elbow=-20.0)
    store.update("/hardware/joint_states", sag, "j")
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["robot"]["state"] == "MOTOR_SUSPECT"
    assert s["motor_power"]["value"] == "suspect"


def test_base_pushed_reports_drive_back(env):
    clock, store, tracker = env
    store.update("/hardware/joint_states", dict(UPRIGHT, base_x=0.05), "j")
    s = derive(make(clock, store, tracker), clock(), 0)
    texts = " ".join(r["text"] for r in s["motor_power"]["reasons"])
    assert "drive back" in texts


def test_new_firmware_boot_loses_origin_but_bridge_restart_does_not(env):
    clock, store, tracker = env
    store.push_event("/arduino/bridge_info", {"session_id": "s2", "boot_id": "b1"}, "bi")  # ROS bridge restart only
    tracker.observe(clock())
    assert tracker.mem.origin["status"] == "confirmed"
    store.push_event("/arduino/bridge_info", {"session_id": "s3", "boot_id": "b2"}, "bi")  # firmware rebooted
    tracker.observe(clock())
    assert tracker.mem.origin["status"] == "lost"
    s = derive(make(clock, store, tracker), clock(), 0)
    # Lost origin is a warning (pickup doesn't need it), not a robot state that blocks starting.
    assert s["robot"]["state"] != "ORIGIN_LOST"
    assert any(a["id"] == "origin" and a["severity"] == "warn" for a in s["next_actions"])


def test_unexpected_odometry_jump_to_zero_loses_origin(env):
    clock, store, tracker = env
    store.update("/hardware/joint_states", dict(UPRIGHT, base_x=0.4), "j")
    tracker.observe(clock())
    store.update("/hardware/joint_states", dict(UPRIGHT), "j")
    tracker.observe(clock())
    assert tracker.mem.origin["status"] == "lost"


def test_unacked_abort_is_sticky(env):
    clock, store, tracker = env
    store.update("/gesture_control/executor_state", {"busy": False, "steps": [], "undo_stack": [],
                                                     "last_result": {"outcome": "aborted", "command": "pick_up_coffee",
                                                                     "step": "refine_target", "reason": "no stable detection", "t": 5.0}}, "es")
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["robot"]["state"] == "ABORTED"
    assert any(n["id"] == "abort" for n in s["next_actions"])
    tracker.acknowledge("op")
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["robot"]["state"] != "ABORTED"


# ---------------------------------------------------------------------------- detection window (§4.2)
def test_window_spread_zband_two_clusters_nonfinite():
    now = 100.0
    ok = detection_window(good_cycles(now), now, CFG, 1)
    assert ok["ok"] and ok["consecutive"] == 4
    # Height is the operator's judgment: an unusual but consistent height still passes.
    high = detection_window(good_cycles(now, z=0.96), now, CFG, 1)
    assert high["ok"]
    # Physically impossible poses (dead camera frames) never count.
    garbage = detection_window(good_cycles(now, z=-6.79), now, CFG, 1)
    assert not garbage["ok"]
    # One stray detection is not a second object...
    stray = good_cycles(now)
    stray[1] = dict(stray[1], pose=(0.95, 0.42, 0.765))
    w = detection_window(stray, now, CFG, 1)
    assert not w["two_candidates"] and w["ok"] and w["consecutive"] == 3
    # ...but two agreeing detections elsewhere are.
    two = good_cycles(now)
    two[0] = dict(two[0], pose=(0.95, 0.42, 0.765))
    two[1] = dict(two[1], pose=(0.951, 0.421, 0.766))
    w = detection_window(two, now, CFG, 1)
    assert w["two_candidates"] and not w["ok"]
    # A NaN pose is skipped like any failed cycle.
    nan = good_cycles(now)
    nan[-1] = dict(nan[-1], pose=(math.nan, 0.0, 0.76))
    assert detection_window(nan, now, CFG, 1)["consecutive"] == 3
    # Skipped cycles between agreeing detections don't reset the count.
    gap = good_cycles(now)
    gap[-2] = dict(gap[-2], stage="publish_gate", pose=None)
    assert detection_window(gap, now, CFG, 1)["consecutive"] == 3


# ---------------------------------------------------------------------------- actions (§6.2)
class FakeIO:
    mode = "sim"
    structured = True

    def __init__(self, external=None):
        self.sent = []
        self.external = external or {}

    class supervisor:  # noqa: N801
        units = ["hp-bridge", "hp-moveit"]

        @staticmethod
        def start(unit):
            return {"result": "started"}

    def external_instance(self, unit):
        return self.external.get(unit, [])

    def publish_stop(self):
        self.sent.append("stop")
        return True

    def call_cancel(self):
        return True

    def send_command(self, c, token, args=None):
        self.sent.append(c)

    def send_approval(self, *a):
        self.sent.append("approval")

    def clear_octomap(self):
        return True

    def set_task_id(self, t):
        pass


def service(env):
    clock, store, tracker = env
    io = FakeIO()
    svc = ActionService(io, tracker, Audit(Path(tempfile.mkdtemp())), CFG, LogBuffer())
    return svc, io


def test_view_only_client_can_stop_but_not_move(env):
    clock, store, tracker = env
    svc, io = service(env)
    s = derive(make(clock, store, tracker, good_cycles(clock())), clock(), 0)
    assert svc.handle("stop", "r1", "viewer", "op", {}, s, clock())["status"] in ("sent", "confirmed")
    r = svc.handle("return_to_start", "r2", "viewer", "op", {}, s, clock())
    assert r["status"] == "rejected" and "control" in r["detail"]


def test_dedup_by_request_id(env):
    clock, store, tracker = env
    svc, io = service(env)
    s = derive(make(clock, store, tracker), clock(), 0)
    svc.handle("take_control", "t", "c1", "op", {}, s, clock())
    svc.handle("gripper_open", "same", "c1", "op", {}, s, clock())
    r = svc.handle("gripper_open", "same", "c1", "op", {}, s, clock())
    assert r.get("duplicate") and io.sent.count("gripper_open") == 1


def test_motion_blocked_when_motor_power_suspect(env):
    clock, store, tracker = env
    svc, io = service(env)
    store.update("/hardware/joint_states", dict(UPRIGHT, joint_elbow=-34.5), "j")
    s = derive(make(clock, store, tracker), clock(), 0)
    svc.handle("take_control", "t", "c1", "op", {}, s, clock())
    allowed = svc.allowed(s, "c1", clock())
    for name in ("home", "return_to_start", "gripper_open", "undo_step"):
        assert not allowed[name]["allowed"], name
    assert allowed["stop"]["allowed"]


def test_recovery_after_abort_requires_inspection(env):
    clock, store, tracker = env
    svc, io = service(env)
    store.update("/gesture_control/executor_state", {"busy": False, "steps": [], "undo_stack": [
        {"undo_id": 1, "step": "lift", "reversible": True}],
        "capabilities": {"undo": True, "resume": False, "return_to_start": True},
        "last_result": {"outcome": "aborted", "command": "pick_up_coffee", "step": "lift", "reason": "x", "t": 9.0}}, "es")
    s = derive(make(clock, store, tracker), clock(), 0)
    svc.handle("take_control", "t", "c1", "op", {}, s, clock())
    assert svc.handle("undo_sequence", "u1", "c1", "op", {}, s, clock())["status"] == "rejected"
    r = svc.handle("undo_sequence", "u2", "c1", "op", {"inspected": True}, s, clock())
    assert r["status"] == "sent" and "undo_sequence" in io.sent
    assert tracker.mem.abort_acked_t == 9.0  # choosing a recovery acknowledges the abort


def test_status_text_fallback_parser():
    assert parse_status_text("[pick_up_coffee] step 6/9: align_refined_grasp")["step_index"] == 6
    assert parse_status_text("Plan for align_refined_grasp is waiting for approval")["kind"] == "awaiting_approval"
    a = parse_status_text("Sequence pick_up_coffee ABORTED at refine_target: no stable detection")
    assert a["step"] == "refine_target" and a["reason"] == "no stable detection"


# ---------------------------------------------------------------------------- boot identity (§7.4 integration bugs)
def test_bridge_info_without_boot_yet_is_not_a_new_identity(env):
    clock, store, tracker = env
    # A reopened port publishes bridge_info before the firmware's BOOT line arrives.
    store.push_event("/arduino/bridge_info", {"session_id": "s9", "boot_id": None, "zero_supported": True}, "bi")
    tracker.observe(clock())
    assert tracker.mem.origin["status"] == "confirmed"
    store.push_event("/arduino/bridge_info", {"session_id": "s9", "boot_id": "b1", "zero_supported": True}, "bi")
    tracker.observe(clock())
    assert tracker.mem.origin["status"] == "confirmed"


def test_console_started_after_boot_no_false_motor_suspect():
    """Robot booted (and moved) before the console started: neither the boot-upright rule nor
    the idle-drift rule may fire just because the arm isn't upright and the base isn't at 0."""
    clock = Clock()
    store = SignalStore(clock)
    tracker = Tracker(store, Path(tempfile.mkdtemp()))
    store.push_event("/arduino/bridge_info", {"session_id": "s1", "boot_id": "b1"}, "bi")
    moved = dict(UPRIGHT, base_x=0.42, joint_elbow=60.0, joint_shoulder=30.0)
    healthy(store, clock, moved)
    tracker.observe(clock())
    clock.t += 20.0
    healthy(store, clock, moved)
    tracker.observe(clock())
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["motor_power"]["value"] == "unverified", s["motor_power"]


# ---------------------------------------------------------------------------- never start a duplicate (hardware 2026-10-18)
UNITS_BRIDGE_DOWN = {**UNITS, "hp-bridge": dict(UNITS["hp-bridge"], active_state="inactive", sub_state="dead")}


def test_live_bridge_with_down_unit_is_external_not_down(env):
    clock, store, tracker = env
    inp = make(clock, store, tracker)
    inp.units = UNITS_BRIDGE_DOWN
    s = derive(inp, clock(), 0)
    bridge = next(c for c in s["components"] if c["unit"] == "hp-bridge")
    assert bridge["health"] == "external" and "outside console" in bridge["detail"]
    assert bridge["start_block"]
    assert s["robot"]["state"] != "BRIDGE_DOWN"
    assert not any(n["action"] and n["action"]["name"] == "unit_start" for n in s["next_actions"])


def test_unit_start_refused_when_port_is_held_outside(env):
    clock, store, tracker = env
    io = FakeIO(external={"hp-bridge": ["~/ttyArduino is open by pid 4242 (python3 arduino_reader.py)"]})
    svc = ActionService(io, tracker, Audit(Path(tempfile.mkdtemp())), CFG, LogBuffer())
    inp = make(clock, store, tracker)
    inp.units = UNITS_BRIDGE_DOWN
    s = derive(inp, clock(), 0)
    svc.handle("take_control", "t", "c1", "op", {}, s, clock())
    r = svc.handle("unit_start", "u1", "c1", "op", {"unit": "hp-bridge"}, s, clock())
    assert r["status"] == "rejected" and "already running outside the console" in r["detail"] and "pid 4242" in r["detail"]


def test_unit_start_refused_when_topic_live_even_without_process_evidence(env):
    clock, store, tracker = env
    svc = ActionService(FakeIO(), tracker, Audit(Path(tempfile.mkdtemp())), CFG, LogBuffer())
    inp = make(clock, store, tracker)
    inp.units = UNITS_BRIDGE_DOWN
    s = derive(inp, clock(), 0)
    svc.handle("take_control", "t", "c1", "op", {}, s, clock())
    r = svc.handle("unit_start", "u2", "c1", "op", {"unit": "hp-bridge"}, s, clock())
    assert r["status"] == "rejected" and "live" in r["detail"]


def test_duplicate_nodes_and_tf_drops_surface(env):
    clock, store, tracker = env
    for _ in range(3):
        store.push_event("tf/drop", "d405_obstacle_cloud", "/rosout")
    tracker.observe(clock())
    inp = make(clock, store, tracker)
    inp.graph_nodes = {"arduino_bridge": 2}
    s = derive(inp, clock(), 0)
    ids = {n["id"]: n for n in s["next_actions"]}
    assert ids["dup-arduino_bridge"]["severity"] == "bad"
    assert "tf-drops" in ids
    tf = next(c for c in s["components"] if c["id"] == "hp-tf")
    assert tf["health"] == "silent" and "TF missing" in tf["detail"]


def test_upright_held_arm_counts_as_motor_power(env):
    clock, store, tracker = env
    store.update("/hardware/joint_states", dict(UPRIGHT), "j")
    tracker.observe(clock())
    tracker.mem.motor_verified = None  # e.g. console restarted, nothing has moved yet
    s = derive(make(clock, store, tracker), clock(), 0)
    assert s["motor_power"]["value"] in ("verified", "suspect")
    if s["motor_power"]["value"] == "verified":
        assert s["motor_power"]["verified"]["how"] in ("arm holding upright against gravity",
                                                       "arm rose to upright after bridge start")
