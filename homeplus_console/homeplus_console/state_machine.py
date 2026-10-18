"""derive_state(inputs, now) -> ConsoleState. Pure: same inputs, same output (UI_PLAN §3).

Priority order: the most dangerous matching condition wins the robot headline.
Every displayed value is three-valued; stale values become None with last_value and age kept.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations
from typing import Any, Dict, List, Optional

from .signals import Signal, sig_view
from .tracker import ERROR_HINTS, Memory, error_code, finite

SEV_ORDER = {"bad": 0, "warn": 1, "info": 2, "ok": 3, "unknown": 1}

COMPONENTS = [
    # id, label, unit, node name (for last_error)
    ("hp-tf", "Robot TF", "hp-tf", "robot_state_publisher"),
    ("hp-camera-rx", "D405 camera", "hp-camera-rx", "d405_vm_receiver"),
    ("hp-vision", "Detector", "hp-vision", "molmopoint"),
    ("hp-moveit", "MoveIt", "hp-moveit", "gesture_motion_executor"),
    ("hp-bridge", "Arduino", "hp-bridge", "arduino_bridge"),
    ("d455", "D455 camera", None, "d455"),
    ("gpu", "GPU server", None, "gpu"),
]

FEEDS = [
    # id, label, topic signal, kind, stale threshold key
    ("d405_overlay", "D405 · detector overlay", "img/d405_overlay", "overlay", "overlay_stale_sec"),
    ("d405_color", "D405 color", "img/d405_color", "color", "camera_fresh_sec"),
    ("d405_depth", "D405 depth", "img/d405_depth", "depth", "camera_fresh_sec"),
    ("d455_color", "D455 overview", "img/d455_color", "color", "camera_fresh_sec"),
]
FEED_EXPECTED_FPS = {"d405_overlay": 0.5, "d405_color": 5.0, "d405_depth": 5.0, "d455_color": 5.0}


@dataclass
class Inputs:
    signals: Dict[str, Signal]
    mem: Memory
    units: Dict[str, Dict[str, Any]]
    frozen_for: Dict[str, float]      # feed -> seconds the thumbnail hash has been unchanged
    frame_seq: Dict[str, Optional[int]]
    logs_recent: List[Dict[str, Any]]  # newest last, any node
    cfg: Dict[str, Any]
    structured: bool
    mode: str
    flow: Optional[Dict[str, Any]]
    requested_task: int
    foreign_publishers: List[str]
    external: Dict[str, List[str]] = None       # unit → evidence it runs outside the console
    graph_nodes: Dict[str, int] = None          # ROS node name → count (duplicates > 1)


def _age(sig: Optional[Signal], now: float) -> Optional[float]:
    return sig.age(now) if sig else None


def _fresh(sig: Optional[Signal], now: float, max_age: float) -> bool:
    return bool(sig) and sig.fresh(now, max_age)


def _unit_active(units: Dict[str, Dict[str, Any]], unit: str) -> bool:
    """A unit the profile doesn't manage counts as running: its health comes from observed signals alone."""
    if unit not in units:
        return True
    return units[unit].get("active_state") == "active"


def _last_log(logs: List[Dict[str, Any]], node: str, levels=("WARN", "ERROR", "FATAL")) -> Optional[Dict[str, Any]]:
    for line in reversed(logs):
        if line["node"] == node and line["level"] in levels:
            return line
    return None


def fmt_age(sec: Optional[float]) -> str:
    if sec is None:
        return "never"
    if sec < 10:
        return f"{sec:.1f} s"
    if sec < 120:
        return f"{sec:.0f} s"
    return f"{sec / 60:.0f} min"


# ---------------------------------------------------------------------------- perception
def detection_window(cycles, now: float, cfg: Dict[str, Any], task_id: int) -> Dict[str, Any]:
    recent = [c for c in cycles if now - c["t"] <= cfg["window_max_age_sec"]][-cfg["window_n"]:]
    samples = [{"x": c["pose"][0], "y": c["pose"][1], "z": c["pose"][2], "age": round(now - c["t"], 1)}
               for c in recent if c["pose"] is not None]
    # Largest group of recent detections that agree within the spread limit. Skipped
    # cycles (rgbd skew, stale frames) between them do not reset the count, matching the
    # executor's own stability gate. Height plausibility is left to the operator.
    done = [c["pose"] for c in recent
            if c["stage"] == "done" and c["pose"] is not None and all(finite(v) for v in c["pose"])
            and -0.2 <= c["pose"][2] <= 3.0]  # physically impossible (e.g. z = -6.79 m from dead frames)
    pts = []
    for anchor in done:
        group = [p for p in done if math.dist(anchor, p) < cfg["window_spread_m"]]
        if len(group) > len(pts):
            pts = group
    run = pts
    all_finite = True
    spread = max((math.dist(a, b) for a, b in combinations(pts, 2)), default=0.0)
    # two candidate objects: any pair of window samples farther apart than the cluster distance
    all_pts = [(s["x"], s["y"], s["z"]) for s in samples if all(finite(v) for v in (s["x"], s["y"], s["z"]))]
    two = False
    if len(all_pts) >= 2:
        far = max(math.dist(a, b) for a, b in combinations(all_pts, 2))
        # A second object needs its own agreeing pair; one stray detection is not a second cup.
        others = [p for p in all_pts if not pts or math.dist(p, pts[0]) > cfg["window_cluster_m"]]
        two = far > cfg["window_cluster_m"] and any(
            math.dist(a, b) < cfg["window_spread_m"] for a, b in combinations(others, 2))
    n_ok = len(run) >= cfg["window_min_consecutive"]
    ok = n_ok and not two
    reasons = []
    if not n_ok:
        reasons.append(f"{len(run)}/{cfg['window_min_consecutive']} agreeing detections "
                       f"(within {cfg['window_spread_m'] * 100:.0f} cm)")
    if two:
        reasons.append("two candidate objects")
    last = pts[-1] if pts else (all_pts[-1] if all_pts else None)
    return {"ok": ok, "consecutive": len(run), "required": cfg["window_min_consecutive"],
            "spread_cm": round(spread * 100, 1) if spread is not None and len(pts) >= 2 else None,
            "z_band": list(cfg["z_band"]),  # display only; height is the operator's judgment
            "z": round(last[2], 3) if last else None,
            "position": [round(v, 3) for v in last] if last else None,
            "two_candidates": two, "samples": samples, "reasons": reasons, "task_id": task_id}


def derive_perception(inp: Inputs, now: float) -> Dict[str, Any]:
    S, cfg, units = inp.signals, inp.cfg, inp.units
    color, depth = S.get("img/d405_color"), S.get("img/d405_depth")
    diag = S.get("/molmopoint/diagnostics")
    task = S.get("/molmopoint/task_id")
    rx_active = _unit_active(units, "hp-camera-rx")
    color_age, depth_age = _age(color, now), _age(depth, now)
    frozen = inp.frozen_for.get("d405_color", 0.0)
    diag_age = _age(diag, now)
    d = diag.value if diag else {}
    window = detection_window(inp.mem.cycles, now, cfg, inp.requested_task)
    task_id = task.value if _fresh(task, now, 6.0) else None
    base = {"diagnostics": {"stage": d.get("stage"), "reason": d.get("reason"), "detail": d.get("detail"),
                            "frame_age_sec": d.get("frame_age_sec"), "cycle_sec": d.get("cycle_sec"),
                            "backend": d.get("backend"), "model_text": d.get("model_text"),
                            "age": diag_age, "fresh": diag_age is not None and diag_age < cfg["detector_slow_sec"]},
            "window": window, "task_id": task_id, "requested_task": inp.requested_task}

    def out(state, title, sev, detail, fix=None):
        return {**base, "state": state, "title": title, "severity": sev, "detail": detail, "fix": fix}

    worst = max(a for a in (color_age, depth_age) if a is not None) if color_age is not None and depth_age is not None else None
    if worst is None or worst > cfg["camera_fresh_sec"]:
        why = ("no frames received yet" if worst is None else f"no frames for {fmt_age(worst)}") + (
            " (receiver unit hp-camera-rx is not running)" if not rx_active else "")
        sender = ("Restart the camera sender on the Mac (sudo .venv/bin/python mac_sender.py --fps 5), then check "
                  "the D405 USB-C adapter." if cfg.get("remote_gpu", True) else "Restart hp-camera, then check the D405 USB cable.")
        return out("CAMERA_DOWN", "Camera down", "bad", f"D405 {why}.", sender if rx_active else "Start hp-camera-rx.")
    if frozen >= cfg["camera_frozen_sec"]:
        return out("CAMERA_FROZEN", "Camera frozen", "bad",
                   f"D405 frames identical for {fmt_age(frozen)} while still arriving: the sender is hung.",
                   "Restart the camera sender on the Mac (mac_sender.py).")
    if diag_age is None or diag_age > cfg["detector_down_sec"]:
        return out("DETECTOR_DOWN", "Detector down", "bad",
                   f"No detector diagnostics for {fmt_age(diag_age)}"
                   + (" (hp-vision is not running)." if not _unit_active(units, "hp-vision") else "."),
                   "Restart hp-vision.")
    if cfg.get("remote_gpu", True):
        gpu = S.get("gpu/health")
        if not _fresh(gpu, now, cfg["gpu_fresh_sec"]):
            return out("GPU_DOWN", "GPU server down", "bad",
                       f"/health not answered for {fmt_age(_age(gpu, now))}.",
                       "Check the AWS instance and the SSH tunnel to 192.168.64.1:8765.")
    if diag_age > cfg["detector_slow_sec"]:
        return out("DETECTOR_SLOW", "Detector slow", "warn", f"Last detector cycle {fmt_age(diag_age)} ago.", None)
    stage, reason = d.get("stage"), d.get("reason")
    if stage == "inputs":
        return out("DETECTOR_WAITING", "Detector waiting", "warn", f"inputs: {reason}", None)
    if stage in ("motion_gate", "publish_gate"):
        # frame_stale / frame_before_motion_done lasting > 10 s while the arm is still: the 7-minute-stale failure
        still = S.get("/arduino/status")
        arm_still = _fresh(S.get("/hardware/joint_states"), now, cfg["joints_fresh_sec"]) and bool(still and still.value)
        run = 0.0
        for c in reversed(inp.mem.cycles):
            if c["reason"] in ("frame_stale", "frame_before_motion_done"):
                run = now - c["t"]
            else:
                break
        if reason in ("frame_stale", "frame_before_motion_done") and arm_still and run >= cfg["detector_old_frames_sec"]:
            return out("DETECTOR_OLD_FRAMES", "Detector is looking at old frames", "bad",
                       f"{stage}: {reason} for {fmt_age(run)} while the arm is still.",
                       "Restart the camera sender, then wait for fresh detections.")
        return out("DETECTOR_PAUSED", "Detector paused", "info", f"{stage}: {reason}", None)
    if stage == "geometry" or reason == "model_returned_no_points":
        said = d.get("model_text") or d.get("detail")
        return out("NO_TARGET", "No target", "warn",
                   f"Model said “{said}”" if said else f"geometry: {reason}",
                   "Check the cup is in view of the hand camera.")
    if inp.requested_task and task_id is not None and task_id != inp.requested_task:
        return out("TASK_MISMATCH", "Detector on wrong task", "warn",
                   f"Detector task_id is {task_id}, requested {inp.requested_task}.", None)
    if not window["ok"]:
        two = window["two_candidates"]
        return out("TARGET_UNSTABLE", "Two candidate objects" if two else "Target unstable", "warn",
                   "; ".join(window["reasons"]) or "waiting for detections",
                   "Remove the second red object from view." if two else None)
    return out("TARGET_OK", "Target OK", "ok",
               f"{window['consecutive']} consecutive · spread {window['spread_cm']} cm · z {window['z']:.3f} m", None)


# ---------------------------------------------------------------------------- feeds and components
def derive_feeds(inp: Inputs, now: float) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    order = [f for f in FEEDS if f[3] != "overlay"] + [f for f in FEEDS if f[3] == "overlay"]
    for fid, label, sname, kind, key in order:
        sig = inp.signals.get(sname)
        age = _age(sig, now)
        stale_after = inp.cfg[key]
        frozen = inp.frozen_for.get(fid, 0.0)
        fps = sig.rate_hz(now, max(5.0, 3.0 / FEED_EXPECTED_FPS[fid])) if sig else 0.0
        note = None
        diag = inp.signals.get("/molmopoint/diagnostics")
        src_age = (diag.value or {}).get("frame_age_sec") if (kind == "overlay" and diag) else None
        if sig is None:
            status = "off"
        elif kind == "overlay" and src_age is not None and src_age > 3.0 and age <= stale_after:
            status, note = "stale", f"detector is drawing on a frame {fmt_age(src_age)} old"
        elif age > stale_after:
            status = "stale"
        elif kind != "overlay" and frozen >= inp.cfg["camera_frozen_sec"]:
            status = "frozen"
        elif fps < FEED_EXPECTED_FPS[fid] * 0.5:
            status = "slow"
        else:
            status = "live"
        if kind == "overlay" and status in ("live", "slow"):
            # A derived feed is only as live as its source: the detector keeps drawing on a dead camera's frame.
            src = next((f for f in out if f["id"] == "d405_color"), None)
            if src and src["status"] in ("frozen", "stale", "off"):
                status = "frozen" if src["status"] == "frozen" else "stale"
                note = f"source camera {src['status']}: detector is drawing on an old frame"
        out.append({"id": fid, "label": label, "kind": kind, "topic": sig.source if sig else None,
                    "seq": inp.frame_seq.get(fid), "age": age, "fps": round(fps, 1), "status": status,
                    "frozen_sec": round(frozen, 1), "stale_after": stale_after,
                    "strip": sig.strip(now) if sig else "0" * 120,
                    "depth_range_m": [0.15, 1.50] if kind == "depth" else None, "note": note,
                    "source_age": src_age})
    return sorted(out, key=lambda f: [x[0] for x in FEEDS].index(f["id"]))


def derive_components(inp: Inputs, now: float, feeds: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    S, cfg = inp.signals, inp.cfg
    observed = {
        "hp-tf": (S.get("tf/world_base"), cfg["tf_fresh_sec"], "TF world→base_link"),
        "hp-camera-rx": (S.get("img/d405_color"), cfg["camera_fresh_sec"], "/cameras/d405/color"),
        "hp-vision": (S.get("/molmopoint/diagnostics"), cfg["detector_slow_sec"], "/molmopoint/diagnostics"),
        "hp-moveit": (S.get("/gesture_control/executor_state"), cfg["executor_fresh_sec"], "executor_state"),
        "hp-bridge": (S.get("/hardware/joint_states"), cfg["joints_fresh_sec"], "/hardware/joint_states"),
        "d455": (S.get("img/d455_color"), cfg["camera_fresh_sec"], "/cameras/d455/color"),
        "gpu": (S.get("gpu/health"), cfg["gpu_fresh_sec"], "GET /health"),
    }
    out = []
    for cid, label, unit, node in COMPONENTS:
        sig, max_age, src = observed[cid]
        view = sig_view(sig, now, max_age, src)
        u = inp.units.get(unit) if unit else None
        ustate = u["active_state"] if u else None
        ext = (inp.external or {}).get(unit) or [] if unit else []
        unit_down = u is not None and ustate in ("inactive", "failed", None)
        if unit_down and (view["fresh"] or ext):
            # Live, but not from the console's unit (tmux, another launch file, by hand). Never "not running".
            health = "external" if view["fresh"] else "silent"
            detail = "outside console" + ("" if view["fresh"] else f": no {src} for {fmt_age(view['age'])}")
        elif u is not None and ustate == "failed":
            health, detail = "down", f"unit failed ({u.get('result')})"
        elif u is not None and ustate in ("inactive", None):
            health, detail = "down", "not running"
        elif u is not None and ustate == "activating":
            health, detail = "starting", "starting…"
        elif not view["fresh"]:
            health = "silent" if u is not None else ("down" if view["age"] is None or view["age"] > max_age * 3 else "silent")
            detail = ("running but silent: " if u is not None else "") + (
                f"no {src} for {fmt_age(view['age'])}" if view["age"] is not None else f"no {src} yet")
        else:
            health, detail = "ok", ""
        if cid == "hp-tf" and health in ("ok", "external"):
            drops = sum(1 for t in inp.mem.tf_drops if now - t <= 30.0)
            if drops:
                health, detail = "silent", f"{drops} depth frame{'s' if drops != 1 else ''} dropped in 30 s: TF missing"
        if cid == "hp-camera-rx" and health == "ok" and feeds["d405_color"]["status"] == "frozen":
            health, detail = "silent", f"frames frozen {fmt_age(feeds['d405_color']['frozen_sec'])}"
        if cid == "d455" and health == "ok" and feeds["d455_color"]["status"] == "frozen":
            health, detail = "silent", "frames frozen"
        value = None
        if view["fresh"]:
            if cid in ("hp-camera-rx", "d455"):
                value = f"{feeds['d405_color' if cid == 'hp-camera-rx' else 'd455_color']['fps']:.1f} fps"
            elif cid == "gpu":
                value = f"{view['value'].get('latency_s', 0):.1f} s"
            elif cid == "hp-vision":
                value = f"{view['age']:.1f} s"
            else:
                value = f"{view['rate_hz']:.0f} Hz"
        err = _last_log(inp.logs_recent, node)
        out.append({"id": cid, "label": label, "unit": unit, "unit_state": ustate,
                    "sub_state": u.get("sub_state") if u else None, "pid": u.get("pid") if u else None,
                    "uptime_s": u.get("uptime_s") if u else None, "restarts": u.get("restarts") if u else None,
                    "health": health, "detail": detail, "value": value,
                    "observed": {k: view[k] for k in ("age", "fresh", "rate_hz", "strip", "source")},
                    "last_error": {"t": err["t_wall"], "text": err["text"]} if err else None,
                    "observed_only": unit is None, "external": ext,
                    # the server re-checks this at execution time; the UI uses it to disable Start
                    "start_block": (f"Already running outside the console: {ext[0]}" if ext else
                                    (f"Already live: {src} is fresh. Starting another would duplicate it."
                                     if unit_down and view["fresh"] else None))})
    return out


# ---------------------------------------------------------------------------- robot
def derive_motor_power(inp: Inputs, now: float, joints: Optional[Dict[str, float]], ready: bool,
                       executor_idle: Optional[bool]) -> Dict[str, Any]:
    m, cfg = inp.mem, inp.cfg
    s = m.session
    reasons = []
    if joints is not None and s["id"]:
        el, sh = joints["joint_elbow"], joints["joint_shoulder"]
        since = now - s["start_mono"] if s["start_mono"] is not None else 0
        if s.get("observed_boot", True) and not s["upright_reached"] and since >= cfg["boot_upright_sec"]:
            reasons.append({"kind": "boot_upright",
                            "text": f"Arm never rose to upright {since:.0f} s after the bridge started "
                                    f"(elbow {el:.0f}°, shoulder {sh:.0f}°)."})
        if ready and (el <= cfg["folded_elbow_deg"] or sh <= cfg["folded_shoulder_deg"]) and s["upright_reached"]:
            reasons.append({"kind": "folded", "text": f"Arm is resting on its folded limit (elbow {el:.0f}°, shoulder {sh:.0f}°)."})
        held = m.held
        if executor_idle and held and s["upright_reached"]:
            for k, name in (("joint_elbow", "Elbow"), ("joint_shoulder", "Shoulder")):
                if k in held and abs(joints[k] - held[k]) > cfg["drift_arm_deg"]:
                    reasons.append({"kind": "drift_arm",
                                    "text": f"{name} has sagged {abs(joints[k] - held[k]):.0f}° from its held pose."})
                    break
        if executor_idle and held and "base_x" in held:
            d = math.hypot(joints["base_x"] - held["base_x"], joints["base_y"] - held["base_y"])
            dth = abs(joints["base_theta"] - held.get("base_theta", 0.0))
            if d > cfg["drift_base_m"] or dth > cfg["drift_base_deg"]:
                reasons.append({"kind": "drift_base",
                                "text": f"Base is {d * 100:.0f} cm from its held target. "
                                        f"When motors are powered it will drive back."})
    for f in m.faults:
        if not f["acknowledged"] and f["code"] in ("TRACKING_ERROR", "SETTLE_TIMEOUT"):
            reasons.append({"kind": "fault", "text": f"Firmware reported {f['code']}."})
            break
    if reasons:
        return {"value": "suspect", "reasons": reasons, "verified": m.motor_verified}
    if joints is None:
        return {"value": "unknown", "reasons": [{"kind": "no_joints", "text": "No fresh joint states."}],
                "verified": m.motor_verified}
    if m.motor_verified:
        return {"value": "verified", "reasons": [], "verified": m.motor_verified}
    return {"value": "unverified", "reasons": [{"kind": "nothing_moved", "text": "Nothing has moved since the bridge started."}],
            "verified": None}


def bridge_subreason(inp: Inputs, unit_active: bool) -> str:
    if not unit_active:
        st = inp.units.get("hp-bridge", {}).get("active_state")
        return "hp-bridge failed" if st == "failed" else "hp-bridge not running"
    for line in reversed(inp.logs_recent[-80:]):
        if line["node"] != "arduino_bridge":
            continue
        t = line["text"]
        if "serial port is not connected" in t or "could not open port" in t:
            return "Arduino link down" if inp.cfg.get("serial_bridge") else "Arduino USB not attached"
        if "No fresh firmware state feedback" in t:
            return "port open, no firmware telemetry"
        if "Arduino serial failure" in t:
            return "serial error"
    return "no joint states"


def derive(inp: Inputs, now: float, wall: float) -> Dict[str, Any]:
    S, cfg, m = inp.signals, inp.cfg, inp.mem
    feeds = derive_feeds(inp, now)
    feed_map = {f["id"]: f for f in feeds}
    components = derive_components(inp, now, feed_map)
    perception = derive_perception(inp, now)

    js = S.get("/hardware/joint_states")
    joints_age = _age(js, now)
    joints_fresh = joints_age is not None and joints_age <= cfg["joints_fresh_sec"]
    joints = js.value if joints_fresh else None
    bridge_unit = _unit_active(inp.units, "hp-bridge") or bool((inp.external or {}).get("hp-bridge"))
    # Joint telemetry decides; a down unit with live joints is a bridge running outside the console.
    bridge_down = joints_age is None or joints_age > cfg["bridge_down_sec"]
    ready_sig, still_sig = S.get("/arduino/ready"), S.get("/arduino/status")
    # latched values only count while their liveness partner (joints) is fresh (§6.5)
    ready = (bool(ready_sig.value) if _fresh(ready_sig, now, cfg["ready_fresh_sec"]) else None) if joints_fresh else None
    arm_still = (bool(still_sig.value) if still_sig else None) if joints_fresh else None

    es = S.get("/gesture_control/executor_state")
    es_fresh = _fresh(es, now, cfg["executor_fresh_sec"])
    ex = es.value if es_fresh else None
    executor = {"fresh": es_fresh, "age": _age(es, now), "structured": inp.structured,
                "busy": ex.get("busy") if ex else None, "command": ex.get("command") if ex else None,
                "request_token": ex.get("request_token") if ex else None,
                "sequence": ex.get("sequence") if ex else None,
                "steps": (ex or (es.value if es else {}) or {}).get("steps", []),
                "last_result": (ex or (es.value if es else {}) or {}).get("last_result"),
                "last_rejected": ex.get("last_rejected") if ex else None,
                "holding_cup": (ex or (es.value if es else {}) or {}).get("holding_cup"),
                "undo_stack": (ex or {}).get("undo_stack", []),
                "capabilities": (ex or {}).get("capabilities") or {"undo": False, "resume": False, "return_to_start": False},
                "resumable": (ex or {}).get("resumable")}
    idle = (not ex["busy"]) if ex else None

    motor = derive_motor_power(inp, now, joints, bool(ready), idle)
    preview_sig = S.get("/gesture_control/plan_preview")
    approval = None
    if ex and ex.get("approval") and ex.get("sequence", {}) and ex["sequence"].get("phase") == "awaiting_approval":
        ap = ex["approval"]
        preview = preview_sig.value if preview_sig and preview_sig.value.get("plan_id") == ap["plan_id"] else None
        approval = {"plan_id": ap["plan_id"], "step": ap["step"], "deadline_wall": ap.get("deadline_wall"),
                    "remaining_sec": ap.get("remaining_sec"), "timeout_sec": ap.get("timeout_sec", 60.0),
                    "state_age": _age(es, now), "preview": preview, "preview_ready": preview is not None}

    lr = executor["last_result"]
    abort_unacked = bool(lr and lr.get("outcome") in ("aborted", "failed", "cancelled")
                         and lr.get("t") != m.abort_acked_t)
    faults_open = [f for f in m.faults if not f["acknowledged"]]
    origin = dict(m.origin)

    # ---------------- preflight (§5.2)
    def item(id_, label, ok, detail="", panel=None):
        return {"id": id_, "label": label, "ok": ok, "detail": detail, "panel": panel}
    foreign = inp.foreign_publishers
    origin_recent = origin["status"] == "confirmed" and origin.get("at") and wall - origin["at"] < cfg["origin_recent_sec"]
    preflight = [
        item("bridge", "Bridge ready, joints fresh", bool(ready) and joints_fresh,
             "" if joints_fresh else f"joints {fmt_age(joints_age)} old", "health"),
        item("still", "Arm still", arm_still is True, "" if arm_still else "arm moving or unknown", "health"),
        item("motors", "Motor power verified", motor["value"] == "verified", motor["value"], "motors"),
        item("faults", "No fault or unacknowledged abort", not faults_open and not abort_unacked, "", "next"),
        item("executor", "Executor idle and fresh", idle is True, "" if es_fresh else "executor_state stale", "timeline"),
        item("moveit", "MoveIt ready", _fresh(S.get("moveit/ready"), now, cfg["moveit_fresh_sec"]), "", "health"),
        item("tf", "TF OK", _fresh(S.get("tf/world_base"), now, cfg["tf_fresh_sec"]), "", "health"),
        item("camera", "Camera fresh, not frozen",
             feed_map["d405_color"]["status"] in ("live", "slow") and feed_map["d405_depth"]["status"] in ("live", "slow"),
             feed_map["d405_color"]["status"], "feeds"),
        item("task", "Detector task matches", perception["task_id"] == inp.requested_task,
             f"task_id {perception['task_id']}", "perception"),
        item("window", "Detection window passes", perception["window"]["ok"], "; ".join(perception["window"]["reasons"]), "perception"),
        item("publishers", "No foreign command publishers", not foreign, ", ".join(foreign), "health"),
    ]
    if cfg.get("remote_gpu", True):
        preflight.append(item("gpu", "GPU /health OK", _fresh(S.get("gpu/health"), now, cfg["gpu_fresh_sec"]), "", "health"))
    declared = [
        {"id": k, "label": label, "ok": m.declared[k] is not None or (k == "at_start_spot" and bool(origin_recent)),
         "declared": m.declared[k], "auto": k == "at_start_spot" and bool(origin_recent) and m.declared[k] is None}
        for k, label in (("one_target", "One target object in view"), ("area_clear", "Area clear"),
                         )]

    # ---------------- robot headline (§3.2), first match wins
    blockers = [p for p in preflight if not p["ok"]] + [d for d in declared if not d["ok"]]
    if bridge_down:
        sub = bridge_subreason(inp, bridge_unit)
        robot = {"state": "BRIDGE_DOWN", "title": "Bridge down", "severity": "bad", "sub": sub,
                 "detail": f"{sub} · joints {fmt_age(joints_age)} old" + (
                     " · sequence outcome UNKNOWN until the executor reports" if ex and ex.get("busy") else "")}
    elif origin["status"] == "lost":
        b = origin.get("base_at_reset") or {}
        at = (f"robot believed it was at x {b.get('base_x', 0):+.2f} m, y {b.get('base_y', 0):+.2f} m at the reset"
              if b else "position at reset unknown")
        robot = {"state": "ORIGIN_LOST", "title": "Odometry origin lost", "severity": "bad",
                 "sub": origin.get("lost_reason"), "detail": f"{origin.get('lost_reason')}; {at}."}
    elif faults_open:
        f = faults_open[-1]
        robot = {"state": "HARDWARE_FAULT", "title": "Hardware fault", "severity": "bad", "sub": f["code"],
                 "detail": f"{f['text']} · {f['hint']}"}
    elif motor["value"] == "suspect":
        robot = {"state": "MOTOR_SUSPECT", "title": "Motor power suspect", "severity": "bad",
                 "sub": motor["reasons"][0]["kind"], "detail": " ".join(r["text"] for r in motor["reasons"])}
    elif approval:
        robot = {"state": "AWAITING_APPROVAL", "title": "Awaiting approval", "severity": "warn",
                 "sub": approval["step"], "detail": f"{ex['command']} · step {ex['sequence']['step_index']}/{ex['sequence']['step_count']} · {approval['step']}"}
    elif ex and ex.get("busy"):
        sq = ex.get("sequence") or {}
        robot = {"state": "RUNNING", "title": "Running", "severity": "info", "sub": sq.get("phase"),
                 "detail": f"{ex['command']} · step {sq.get('step_index')}/{sq.get('step_count')} · {sq.get('step_name')} · {sq.get('phase')}"}
    elif abort_unacked:
        robot = {"state": "ABORTED", "title": "Aborted" if lr["outcome"] == "aborted" else lr["outcome"].capitalize(),
                 "severity": "bad", "sub": lr.get("step"),
                 "detail": f"{lr.get('command')} at {lr.get('step')}: {lr.get('reason')}"}
    elif not es_fresh:
        robot = {"state": "EXECUTOR_UNKNOWN", "title": "Executor unknown", "severity": "unknown", "sub": None,
                 "detail": f"executor_state silent for {fmt_age(_age(es, now))}"}
    elif blockers:
        robot = {"state": "IDLE_NOT_READY", "title": "Idle · not ready", "severity": "warn", "sub": None,
                 "detail": f"{len(blockers)} blocker{'s' if len(blockers) != 1 else ''}: " + ", ".join(b["label"].lower() for b in blockers[:3])}
    else:
        robot = {"state": "IDLE_READY", "title": "Idle · ready", "severity": "ok", "sub": None, "detail": "All preconditions hold."}

    onset = m.state_onsets.get(robot["state"])
    robot["since_wall"] = onset

    # ---------------- next actions: every problem, with its one fix
    nxt: List[Dict[str, Any]] = []

    def add(id_, sev, problem, detail, fix, action=None, flow=None):
        nxt.append({"id": id_, "severity": sev, "problem": problem, "detail": detail, "fix": fix,
                    "action": action, "flow": flow})

    if robot["state"] == "BRIDGE_DOWN":
        sub = robot["sub"]
        if sub == "Arduino link down":
            add("bridge", "bad", "Arduino serial link down",
                f"The bridge can't read $HOME/ttyArduino; joints {fmt_age(joints_age)} old.",
                "Check the serial bridge: mac_serial_server.sh on the Mac and vm_serial_client.sh in the VM, and the "
                "Arduino USB cable on the Mac. If the Mac side reopens the port the Arduino may reset; the console "
                "will notice and ask you to set start again.",
                action={"name": "unit_restart", "args": {"unit": "hp-bridge"}, "label": "Restart hp-bridge"})
        elif sub == "Arduino USB not attached":
            add("bridge", "bad", "Arduino USB not attached",
                f"The bridge can't open the serial port; joints {fmt_age(joints_age)} old.",
                "Plug the Arduino back in. Reopening the port resets the Arduino, so reconnect through the set-start flow.",
                flow="set_start")
        elif sub in ("hp-bridge not running", "hp-bridge failed"):
            if inp.cfg.get("serial_bridge"):
                add("bridge", "bad", "Arduino bridge not running", "No joint states.",
                    "Start hp-bridge. With the serial bridge this doesn't reset the Arduino.",
                    action={"name": "unit_start", "args": {"unit": "hp-bridge"}, "label": "Start hp-bridge"})
            else:
                add("bridge", "bad", "Arduino bridge not running", "No joint states.",
                    "Start the bridge through the set-start flow (opening the port resets odometry).", flow="set_start")
        else:
            add("bridge", "bad", f"Bridge down: {sub}", f"Joints {fmt_age(joints_age)} old.",
                "Check the Arduino USB cable and power, then reconnect through the re-zero flow.", flow="set_start")
        if ex and ex.get("busy"):
            add("unknown_outcome", "bad", "Sequence outcome unknown",
                "Joint feedback stopped during a sequence. The firmware watchdog halts queued motion after 1.5 s.",
                "Use the physical E-stop if the robot is still moving.")
    if origin["status"] == "lost":
        add("origin", "bad", "Odometry origin lost", robot["detail"] if robot["state"] == "ORIGIN_LOST" else origin.get("lost_reason") or "",
            "Odometry is now zero wherever the robot stood. If it's on the start spot, set start here; if not, the "
            "flow walks you through pushing it there first.", flow="set_start")
    elif origin["status"] == "unconfirmed":
        add("origin", "warn", "Odometry origin not confirmed",
            "The console hasn't seen this bridge session start at the start spot.",
            "Set start here once the robot is on the start spot.", flow="set_start")
    for f in faults_open:
        add(f"fault-{f['id']}", "bad", f"Arduino {f['code']}", f["text"], f["hint"], action={"name": "ack_fault", "label": "Acknowledge"})
    if motor["value"] == "suspect":
        kinds = {r["kind"] for r in motor["reasons"]}
        if kinds & {"boot_upright", "folded", "drift_arm", "fault"}:
            add("motors", "bad", "Arm motors not powered", " ".join(r["text"] for r in motor["reasons"] if r["kind"] != "drift_base"),
                "Check the E-stop, the arm power switch and the battery clips.")
        if "drift_base" in kinds:
            add("drift", "bad", "Base pushed off its held target",
                next(r["text"] for r in motor["reasons"] if r["kind"] == "drift_base"),
                "Don't release the E-stop here: the base would drive back. Set start at the new spot first.", flow="set_start")
    if abort_unacked:
        reason = lr.get("reason") or ""
        code = None
        for c in ERROR_HINTS:
            if c in reason:
                code = c
        if code:
            hint = ERROR_HINTS[code]
        elif "rejected by operator" in reason:
            hint = "You rejected this plan. Undo, return to start, or retry once the cause is fixed."
        elif "timed out" in reason and "approval" in reason:
            hint = "Nobody approved within 60 s, so the executor rejected the plan."
        elif "detection" in reason:
            hint = "Check the camera feed and the detection window, then start again."
        else:
            hint = "Open the failed step in the timeline for its log lines."
        add("abort", "bad", f"Sequence {lr['outcome']} at {lr.get('step')}", reason, hint,
            action={"name": "recover", "label": "Recover"})
    if not es_fresh and not bridge_down:
        add("executor", "bad", "Executor not reporting",
            f"executor_state silent for {fmt_age(_age(es, now))}.",
            "Restart hp-moveit (doesn't touch the Arduino).", action={"name": "unit_restart", "args": {"unit": "hp-moveit"},
                                                                      "label": "Restart hp-moveit"})
    if perception["severity"] in ("bad", "warn") and perception["fix"]:
        problem = perception["title"]
        if perception["state"] == "CAMERA_DOWN":
            age = feed_map["d405_color"]["age"]
            problem = f"Camera feed stalled {fmt_age(age)}" if age is not None else "Camera feed down"
        add("perception", perception["severity"], problem, perception["detail"], perception["fix"])
    watched = {"arduino_bridge": "hp-bridge", "molmopoint": "hp-vision", "robot_state_publisher": "hp-tf",
               "move_group": "hp-moveit", "gesture_motion_executor": "hp-moveit"}
    for name, count in (inp.graph_nodes or {}).items():
        if count > 1 and name in watched:
            unit = watched[name]
            managed = _unit_active(inp.units, unit) and unit in inp.units
            add(f"dup-{name}", "bad", f"{count} {name} nodes running",
                "Two processes run the same component" + (
                    "; two Arduino readers on one serial port split every line and can corrupt commands." if name == "arduino_bridge" else "."),
                f"Stop the console's {unit} (the external launch keeps running)." if managed else
                "Stop the extra process (check tmux and other launch files).",
                action={"name": "unit_stop", "args": {"unit": unit}, "label": f"Stop {unit}"} if managed else None)
    for c in components:
        if c["unit"] and c["external"] and c["unit_state"] == "active":
            add(f"twice-{c['id']}", "bad", f"{c['label']} running twice",
                f"The console's {c['unit']} is up and {c['external'][0]}.",
                f"Stop {c['unit']} or the outside process so only one remains.",
                action={"name": "unit_stop", "args": {"unit": c["unit"]}, "label": f"Stop {c['unit']}"})
    tf_drops = sum(1 for t in m.tf_drops if now - t <= 30.0)
    if tf_drops:
        gap = 0.0
        if js is not None:
            times = [t for t in js._recv_times if now - t <= 30.0]
            gap = max((b - a for a, b in zip(times, times[1:])), default=0.0)
        add("tf-drops", "warn", "TF missing: obstacle cloud dropping depth frames",
            f"d405_obstacle_cloud dropped {tf_drops} depth frame{'s' if tf_drops != 1 else ''} in 30 s "
            "(acquisition-time TF still missing after 0.5 s).",
            (f"joint_states had gaps up to {gap:.1f} s in the same window, and robot_state_publisher builds TF from "
             "them: check the Arduino bridge (duplicate readers, serial errors) first." if gap > 0.3 else
             "joint_states were steady, so check robot_state_publisher (hp-tf) and the camera's TF frame."))
    for c in components:
        if c["health"] in ("down", "silent") and not c["external"] and c["unit"] and c["unit"] not in ("hp-bridge",) and not any(
                n["id"] in ("executor", "perception") and c["unit"] in ("hp-moveit", "hp-vision", "hp-camera-rx") for n in nxt):
            add(f"unit-{c['id']}", "warn", f"{c['label']}: {c['detail']}", "",
                f"Start {c['unit']}." if c["health"] == "down" else f"Restart {c['unit']}.",
                action={"name": "unit_start" if c["health"] == "down" else "unit_restart", "args": {"unit": c["unit"]},
                        "label": f"{'Start' if c['health'] == 'down' else 'Restart'} {c['unit']}"})
    nxt.sort(key=lambda n: SEV_ORDER.get(n["severity"], 3))

    recovery = None
    if joints is not None:
        dx, dy = -joints["base_x"], -joints["base_y"]
        dist = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx)) - joints["base_theta"] if dist > 0.005 else 0.0
        bearing = (bearing + 180) % 360 - 180
        recovery = {"pose": {"x": joints["base_x"], "y": joints["base_y"], "theta": joints["base_theta"]},
                    "home_distance_m": round(dist, 3), "home_bearing_deg": round(bearing, 1),
                    "home_turn_deg": round(-joints["base_theta"], 1),
                    "undo_count": sum(1 for r in executor["undo_stack"] if r.get("reversible")),
                    "undo_next": next((r for r in executor["undo_stack"] if r.get("reversible")), None)}

    return {
        "recovery": recovery,
        "robot": robot, "motor_power": motor, "origin": origin, "estop_declaration": m.estop_declaration,
        "perception": perception, "feeds": feeds, "components": components,
        "joints": {"fresh": joints_fresh, "age": joints_age, "values": joints,
                   "last_values": js.value if js else None, "ready": ready, "arm_still": arm_still,
                   "session_id": m.session["id"],
                   "si": (S["/hardware/joint_states_si"].value if joints_fresh and S.get("/hardware/joint_states_si") else None)},
        "executor": executor, "approval": approval,
        "plan_preview": preview_sig.value if preview_sig else None,
        "plan_preview_age": _age(preview_sig, now), "faults": m.faults[-10:],
        "preflight": preflight, "declared": declared, "next_actions": nxt,
        "degraded": None if inp.structured else "executor_state not published: timeline derived from status text",
    }
