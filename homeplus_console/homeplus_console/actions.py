"""Actions: gate → audit → send → verify (UI_PLAN §3.4, §6).

The server re-checks every action against the state derived at execution time.
The UI disables buttons too, but nothing relies on that.
"""
from __future__ import annotations

import json
import time
import uuid
from collections import OrderedDict, deque
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from .robot_io import RobotIO
from .tracker import Tracker

MOTION = {"approve", "start_task", "home", "gripper_open", "gripper_close", "back_up",
          "return_to_start", "undo_step", "undo_sequence", "resume"}
RECOVERY = ("return_to_start", "undo_step", "undo_sequence", "resume")
TASK_COMMANDS = {1: "pick_up_coffee", 2: "press_square_button", 7: "place_cup_on_drip_tray", 8: "press_round_button"}
# Tasks that need the cup in the gripper; every other task refuses to start while holding it
# (the executor enforces both from the planning scene, this only mirrors it in the UI).
HOLDING_TASKS = {7}
BRIDGE_UNIT = "hp-bridge"


class Audit:
    def __init__(self, data_dir: Path):
        self.dir = data_dir / "audit"
        self.recent: Deque[Dict[str, Any]] = deque(maxlen=50)

    def write(self, entry: Dict[str, Any]) -> None:
        entry = {"t_wall": time.time(), "t_mono": time.monotonic(), **entry}
        self.recent.append(entry)
        self.dir.mkdir(parents=True, exist_ok=True)
        with open(self.dir / (time.strftime("%Y-%m-%d") + ".jsonl"), "a") as f:
            f.write(json.dumps(entry, default=str) + "\n")


class Lease:
    """One controlling browser session; others are view-only plus STOP (§6.3)."""

    def __init__(self, expiry: float):
        self.expiry = expiry
        self.holder: Optional[str] = None
        self.label: Optional[str] = None
        self.last_beat: float = 0.0
        self.lost_at: Optional[float] = None

    def beat(self, client: str, now: float) -> None:
        if client == self.holder:
            self.last_beat = now

    def valid(self, now: float) -> bool:
        ok = self.holder is not None and now - self.last_beat <= self.expiry
        if not ok and self.holder is not None:
            self.lost_at = self.lost_at or now
            self.holder = None
            self.label = None
        if ok:
            self.lost_at = None
        return ok

    def is_holder(self, client: str, now: float) -> bool:
        return self.valid(now) and self.holder == client


class Flow:
    """Guided "Set start here" (§5.3). Each step observes before it advances.

    method "zero": the §7.10 ZERO command (no reset, no arm twitch).
    method "reset": reset the Arduino (DTR / restart mac_serial_server.sh); the arm re-homes upright.
    The push branch adds E-stop, push and release steps for when the base must be moved by hand.
    """

    TITLES = {
        "precheck": "Executor idle",
        "where": "Choose the start spot",
        "estop": "Press the E-stop",
        "push": "Push the robot to the new start spot",
        "zero": "Zero odometry",
        "crosscheck": "Check the E-stop declaration",
        "release": "Release the E-stop",
        "verify": "Verify the base reads (0, 0, 0)",
        "done": "Start set",
    }

    def __init__(self, reason: str, by: str, now: float, method: str):
        self.id = uuid.uuid4().hex[:6]
        self.kind = "set_start"
        self.reason = reason
        self.by = by
        self.method = method
        self.push = False
        self.order = ["precheck", "where", "zero", "verify", "done"]
        self.idx = 0
        self.t_step = now
        self.status = "active"   # active | error | done | cancelled
        self.notes: Dict[str, Dict[str, Any]] = {}
        self.error: Optional[str] = None
        self.crosscheck: Optional[str] = None  # "consistent" | "contradicted"
        self.released_at: Optional[float] = None
        self.sent = False

    def choose_push(self, push: bool) -> None:
        self.push = push
        order = ["precheck", "where"] + (["estop", "push"] if push else []) + ["zero"]
        if push and self.method == "reset":
            order.append("crosscheck")
        order += (["release"] if push else []) + ["verify", "done"]
        self.order = order

    @property
    def step(self) -> str:
        return self.order[self.idx]

    def advance(self, now: float, note: Optional[Dict[str, Any]] = None) -> None:
        if note:
            self.notes[self.step] = note
        self.idx = min(self.idx + 1, len(self.order) - 1)
        self.t_step = now
        self.error = None
        self.sent = False
        if self.step == "done":
            self.status = "done"

    def view(self, now: float, waiting_for: str, options: List[Dict[str, Any]]) -> Dict[str, Any]:
        title = dict(self.TITLES)
        title["zero"] = "Zero odometry (ZERO command)" if self.method == "zero" else "Zero odometry (Arduino reset)"
        return {"id": self.id, "kind": self.kind, "reason": self.reason, "status": self.status, "method": self.method,
                "push": self.push, "step": self.step, "step_index": self.idx,
                "steps": [{"id": s, "title": title[s], "status": ("done" if i < self.idx or self.status == "done" else
                                                                  ("error" if i == self.idx and self.error else
                                                                   ("active" if i == self.idx else "pending"))),
                           "note": self.notes.get(s)} for i, s in enumerate(self.order)],
                "waiting_for": waiting_for, "options": options, "can_confirm": bool(options),
                "confirm_label": options[0]["label"] if options else None,
                "error": self.error, "crosscheck": self.crosscheck, "step_age": now - self.t_step}


class ActionService:
    def __init__(self, io: RobotIO, tracker: Tracker, audit: Audit, cfg: Dict[str, Any], logs):
        self.io, self.tracker, self.audit, self.cfg, self.logs = io, tracker, audit, cfg, logs
        self.lease = Lease(cfg["lease_expiry_sec"])
        self._seen: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()   # request_id -> result (dedup)
        self.tracked: Deque[Dict[str, Any]] = deque(maxlen=20)
        self.flow: Optional[Flow] = None
        self.stop_result: Optional[Dict[str, Any]] = None
        self.requested_task = 1
        self._no_controller_since: Optional[float] = None

    # ------------------------------------------------------------------ gating
    def allowed(self, state: Dict[str, Any], client: Optional[str], now: float) -> Dict[str, Dict[str, Any]]:
        """{action: {allowed, reason, motion}} for this client, from the derived state."""
        robot = state["robot"]["state"]
        ex = state["executor"]
        ap = state["approval"]
        idle = ex["fresh"] and ex["busy"] is False
        joints_ok = state["joints"]["fresh"] and state["joints"]["ready"] is True
        controlling = client is not None and self.lease.is_holder(client, now)
        open_faults = any(not f["acknowledged"] for f in state["faults"])
        motor = state["motor_power"]["value"]
        motor_ok = motor not in ("suspect", "unknown")
        motor_why = f"motor power {motor}: check the E-stop, power switch and battery clips"
        caps = ex.get("capabilities") or {}
        rec = state.get("recovery") or {}
        flow_busy = self.flow is not None and self.flow.status in ("active", "error")
        out: Dict[str, Dict[str, Any]] = {}

        def put(name, ok, reason=None, needs_control=True):
            if ok and needs_control and not controlling:
                ok, reason = False, "view only: take control first"
            out[name] = {"allowed": bool(ok), "reason": None if ok else reason, "motion": name in MOTION}

        put("stop", True, needs_control=False)
        put("take_control", not controlling, "you have control", needs_control=False)
        put("release_control", controlling, "you don't have control", needs_control=False)
        put("approve", ap is not None and ap["preview_ready"], "no plan awaiting approval" if ap is None else
            "plan preview for this plan_id has not arrived")
        put("reject", ap is not None, "no plan awaiting approval")
        put("cancel", ex["fresh"] and bool(ex["busy"]), "no sequence running")
        start_ok = robot == "IDLE_READY" and state["perception"]["state"] == "TARGET_OK" and not flow_busy
        start_reason = None
        if not start_ok:
            if robot == "IDLE_NOT_READY":
                start_reason = state["robot"]["detail"]
            elif robot != "IDLE_READY":
                start_reason = f"robot is {state['robot']['title'].lower()}"
            elif state["perception"]["state"] != "TARGET_OK":
                start_reason = f"perception: {state['perception']['title'].lower()}"
            else:
                start_reason = "set-start flow in progress"
        put("start_task", start_ok, start_reason)
        out["start_task"]["tasks"] = {t: self._task_gate(t, ex.get("holding_cup")) for t in TASK_COMMANDS}
        def why_motion(extra_ok=True, extra_why=""):
            if not idle:
                return False, "executor not idle"
            if not joints_ok:
                return False, "bridge not ready (joints stale)"
            if not motor_ok:
                return False, motor_why
            if flow_busy:
                return False, "set-start flow in progress"
            return extra_ok, extra_why

        origin = state["origin"]["status"]
        ok, why = why_motion(not open_faults and origin == "confirmed",
                             "hardware fault open" if open_faults else f"origin {origin}: zero is not the start spot")
        put("home", ok, why)
        put("return_to_start", ok and caps.get("return_to_start", True), why or "executor can't return to start")
        ok, why = why_motion()
        put("gripper_open", ok, why)
        put("gripper_close", ok, why)
        put("back_up", ok, why)  # relative to the measured base pose: no confirmed origin needed
        # Choosing a task points the detector at its target right away (no motion).
        put("select_task", idle is not False, "executor busy")
        n_undo = rec.get("undo_count", 0)
        ok, why = why_motion(bool(caps.get("undo")) and n_undo > 0,
                             "nothing to undo" if caps.get("undo") else "executor has no undo (§7.8 not merged)")
        put("undo_step", ok, why)
        put("undo_sequence", ok, why)
        ok, why = why_motion(bool(caps.get("resume")) and bool(ex.get("resumable")) and origin != "lost",
                             "nothing to resume" if caps.get("resume") else "executor can't resume (§7.11)")
        put("resume", ok, why)
        put("clear_octomap", idle, "executor not idle")
        put("ack_fault", open_faults or robot == "ABORTED", "nothing to acknowledge")
        put("declare", idle, "executor not idle")
        put("unit_start", idle or not ex["fresh"], "executor busy")
        put("unit_stop", True)
        put("unit_restart", idle or not ex["fresh"], "executor busy")
        put("recover", robot == "ABORTED", "nothing to recover from")
        put("flow_set_start", not flow_busy and (idle or not ex["fresh"]), "flow already running" if flow_busy else "executor busy")
        put("flow_confirm", self.flow is not None and self.flow.status == "active", "no flow step to confirm")
        put("flow_cancel", self.flow is not None and self.flow.status in ("active", "error"), "no flow running")
        put("flow_retry", self.flow is not None and self.flow.status == "error", "nothing to retry")
        return out

    @staticmethod
    def _task_gate(task: int, holding: Optional[bool]) -> Dict[str, Any]:
        """Whether this task fits the gripper's load. Unknown (no status since the executor
        started) allows both: the executor checks the planning scene and rejects with a reason."""
        if task in HOLDING_TASKS and holding is False:
            return {"allowed": False, "reason": "not holding the cup: pick it up first"}
        if task not in HOLDING_TASKS and holding is True:
            return {"allowed": False, "reason": "holding the cup: place it (or open the gripper) first"}
        return {"allowed": True, "reason": None}

    # ------------------------------------------------------------------ entry point
    def handle(self, name: str, request_id: str, client: str, operator: str, args: Dict[str, Any],
               state: Dict[str, Any], now: float) -> Dict[str, Any]:
        if request_id in self._seen:
            return {**self._seen[request_id], "duplicate": True}
        allowed = self.allowed(state, client, now)
        unit = args.get("unit")
        gate = allowed.get(name)
        if gate is None:
            result = {"status": "rejected", "detail": f"unknown action {name}"}
        elif not gate["allowed"]:
            result = {"status": "rejected", "detail": gate["reason"]}
        elif name == "start_task" and int(args.get("task", 1)) not in TASK_COMMANDS:
            result = {"status": "rejected", "detail": f"unknown task {args.get('task')}"}
        elif name == "start_task" and not gate["tasks"][int(args.get("task", 1))]["allowed"]:
            result = {"status": "rejected", "detail": gate["tasks"][int(args.get("task", 1))]["reason"]}
        elif name in ("unit_start", "unit_restart") and (dup := self._duplicate_risk(name, unit, state)):
            result = {"status": "rejected", "detail": dup}
        elif name in ("unit_start", "unit_restart") and unit == BRIDGE_UNIT and not self.cfg.get("serial_bridge"):
            result = {"status": "rejected",
                      "detail": "Opening the USB port resets the Arduino and its odometry: use the set-start flow."}
        elif name in RECOVERY and state["robot"]["state"] == "ABORTED" and not args.get("inspected"):
            result = {"status": "rejected", "detail": "Confirm you've inspected the robot first."}
        else:
            result = None
        entry = {"operator": operator, "client": client, "action": name, "args": args,
                 "request_id": request_id, "state_before": state["robot"]["state"]}
        if result is not None:
            self.audit.write({**entry, "result": result})
            self._remember(request_id, result)
            return result
        # audit before sending (§6.2)
        self.audit.write({**entry, "result": {"status": "sent"}})
        if name in RECOVERY and state["robot"]["state"] == "ABORTED":
            self.tracker.acknowledge(operator)  # choosing a recovery acknowledges the abort
        result = self._send(name, request_id, client, operator, args, state, now)
        self.audit.write({**entry, "result": result, "phase": "after_send"})
        self._remember(request_id, result)
        return result

    def _duplicate_risk(self, name: str, unit: Optional[str], state: Dict[str, Any]) -> Optional[str]:
        """Never start a second instance of a component (hardware finding 2026-10-18: a console-started
        arduino_reader joined one from a tmux launch on the same pty; the two split every serial line).
        Checked fresh here, at execution time, whatever the UI showed."""
        if not unit:
            return None
        try:
            evidence = self.io.external_instance(unit)
        except Exception as exc:  # noqa: BLE001 (can't tell = don't start)
            return f"Can't verify that {unit} isn't already running ({exc!r}); not starting a possible duplicate."
        comp = next((c for c in state["components"] if c["unit"] == unit), None)
        own_active = bool(comp and comp["unit_state"] in ("active", "activating"))
        if evidence and (name == "unit_start" or not own_active):
            return (f"{unit} is already running outside the console: {evidence[0]}. Starting another would run "
                    "two copies" + (" on one serial port." if unit == BRIDGE_UNIT else ".") +
                    " Use the running one, or stop it first.")
        if comp and not own_active and comp["observed"]["fresh"] and name == "unit_start":
            return (f"{comp['observed']['source']} is live although {unit} is down: something outside the console "
                    f"is running it. Not starting a duplicate.")
        return None

    def _remember(self, rid: str, result: Dict[str, Any]) -> None:
        self._seen[rid] = result
        while len(self._seen) > 500:
            self._seen.popitem(last=False)

    def _track(self, name: str, rid: str, detail: str, check: Callable[[Dict[str, Any]], Optional[Tuple[str, str]]],
               timeout: float, now: float, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        t = {"request_id": rid, "action": name, "args": args or {}, "status": "sent", "detail": detail,
             "t_sent": time.time(), "_t0": now, "_timeout": timeout, "_check": check}
        self.tracked.append(t)
        return {"status": "sent", "detail": detail}

    # ------------------------------------------------------------------ send
    def _send(self, name, rid, client, operator, args, state, now) -> Dict[str, Any]:
        io = self.io
        if name == "stop":
            return self._stop(rid, state, now)
        if name == "take_control":
            prev = self.lease.holder
            self.lease.holder, self.lease.label, self.lease.last_beat = client, operator, now
            return {"status": "confirmed", "detail": f"control taken from {prev}" if prev else "control taken"}
        if name == "release_control":
            self.lease.holder = None
            return {"status": "confirmed", "detail": "control released"}
        if name == "approve":
            pid = args.get("plan_id")
            if pid != state["approval"]["plan_id"]:
                return {"status": "rejected", "detail": f"plan {pid} is no longer the plan awaiting approval"}
            io.send_approval(pid, True, operator)
            return self._track(name, rid, f"approve plan #{pid}",
                               lambda s: ("confirmed", "plan approved; step executing")
                               if not s["approval"] or s["approval"]["plan_id"] != pid else None, 2.0, now, args)
        if name == "reject":
            pid = state["approval"]["plan_id"]
            io.send_approval(pid, False, operator)
            return self._track(name, rid, f"reject plan #{pid}",
                               lambda s: ("confirmed", "plan rejected") if not s["approval"] else None, 2.0, now, args)
        if name == "cancel":
            res = io.call_cancel()
            if res is None:
                io.publish_stop()
            return self._track(name, rid, "cancel sequence",
                               lambda s: ("confirmed", "sequence cancelled") if s["executor"]["busy"] is False else None,
                               3.0, now, args)
        if name in ("start_task", "home", "gripper_open", "gripper_close", "back_up") + RECOVERY:
            command = TASK_COMMANDS[int(args.get("task", 1))] if name == "start_task" else name
            if name == "start_task":
                self.requested_task = int(args.get("task", 1))
                io.set_task_id(self.requested_task)
            token = f"{time.time():.3f}-{rid[:6]}"
            io.send_command(command, token, {k: v for k, v in args.items() if k != "inspected"})

            def check(s, token=token, command=command):
                ex = s["executor"]
                if ex["last_rejected"] and ex["last_rejected"].get("token") == token:
                    return ("not_started", f"NOT STARTED: {ex['last_rejected'].get('reason')}")
                if ex["busy"] and ex["request_token"] == token:
                    return ("confirmed", f"{command} running")
                return None
            return self._track(name, rid, f"{command} sent", check, self.cfg["start_confirm_sec"], now, args)
        if name == "clear_octomap":
            ok = io.clear_octomap()
            return {"status": "confirmed" if ok else "failed", "detail": "octomap cleared" if ok else "service unavailable"}
        if name == "ack_fault":
            done = self.tracker.acknowledge(operator)
            return {"status": "confirmed", "detail": "acknowledged: " + ", ".join(done) if done else "nothing to acknowledge"}
        if name == "declare":
            key = args.get("item")
            if key not in self.tracker.mem.declared:
                return {"status": "rejected", "detail": f"unknown item {key}"}
            val = args.get("value", True)
            self.tracker.mem.declared[key] = {"at": time.time(), "by": operator} if val else None
            return {"status": "confirmed", "detail": f"{key} {'declared' if val else 'cleared'}"}
        if name in ("unit_start", "unit_stop", "unit_restart"):
            unit = args.get("unit")
            if unit not in io.supervisor.units:
                return {"status": "rejected", "detail": f"unknown unit {unit}"}
            if name == "unit_stop" and unit == BRIDGE_UNIT:
                io.publish_stop()
            verb = name.split("_")[1]
            res = getattr(io.supervisor, verb)(unit)
            want = "inactive" if verb == "stop" else "active"
            return self._track(name, rid, f"{verb} {unit}: {res.get('result')}",
                               lambda s: ("confirmed", f"{unit} {want}") if
                               next((c for c in s["components"] if c["unit"] == unit), {}).get("unit_state") == want else None,
                               10.0, now, args)
        if name == "recover":
            return {"status": "rejected", "detail": "choose a recovery action"}
        if name == "flow_set_start":
            method = "zero" if self.tracker.mem.zero_supported else "reset"
            self.flow = Flow(args.get("reason", "operator request"), operator, now, method)
            return {"status": "confirmed", "detail": f"set-start flow started ({method})"}
        if name == "select_task":
            task = int(args.get("task", 1))
            if task not in TASK_COMMANDS:
                return {"status": "rejected", "detail": f"unknown task {task}"}
            self.requested_task = task
            self.io.set_task_id(task)
            return {"status": "confirmed", "detail": f"detector on task {task}"}
        if name == "flow_cancel":
            self.flow.status = "cancelled"
            self.tracker.mem.rezero_expected = False
            self.tracker.mem.zero_expected = False
            return {"status": "confirmed", "detail": "flow cancelled"}
        if name == "flow_retry":
            self.flow.status, self.flow.error, self.flow.t_step = "active", None, now
            self.flow.sent = False
            if self.flow.step == "release":
                self.flow.released_at = None
            return {"status": "confirmed", "detail": "retrying step"}
        if name == "flow_confirm":
            return self._flow_confirm(operator, args, state, now)
        return {"status": "rejected", "detail": f"unhandled action {name}"}

    # ------------------------------------------------------------------ STOP (§6.1)
    def _stop(self, rid: str, state: Dict[str, Any], now: float) -> Dict[str, Any]:
        reached = self.io.publish_stop()
        cancel = self.io.call_cancel()
        was_busy = bool(state["executor"]["busy"])
        if not reached:
            self.stop_result = {"t": time.time(), "status": "not_confirmed", "request_id": rid,
                                "detail": "Bridge down: STOP could not reach the Arduino. The firmware watchdog "
                                          "halts queued motion after 1.5 s without host heartbeat."}
            self.logs.add("console", "WARN", "STOP: bridge down, could not reach the Arduino")
            return {"status": "not_confirmed", "detail": self.stop_result["detail"]}
        self.stop_result = {"t": time.time(), "status": "sent", "request_id": rid,
                            "detail": "STOP sent: verifying motion halted" + (", sequence cancelled" if was_busy else ""),
                            "_t0": now, "_was_busy": was_busy, "_cancel": cancel}
        self.logs.add("console", "WARN", "STOP sent (/arduino/stop ×3" + (", /gesture_control/cancel)" if cancel is not None else ")"))
        return {"status": "sent", "detail": self.stop_result["detail"]}

    def _verify_stop(self, state: Dict[str, Any], now: float) -> None:
        r = self.stop_result
        # A STOP sent while the bridge was down (no "_t0") clears once the bridge
        # reports the arm still again; until then nothing confirms it halted.
        if r and r["status"] == "not_confirmed" and "_t0" not in r and state["joints"]["arm_still"] is True:
            self.stop_result = None
            return
        if not r or r["status"] != "sent":
            return
        t = now - r["_t0"]
        halted = state["joints"]["arm_still"] is True
        idle = state["executor"]["busy"] is False
        if halted and (idle or not r["_was_busy"]):
            r["status"] = "confirmed"
            r["detail"] = "STOP confirmed: motion halted" + (", sequence cancelled" if r["_was_busy"] else "")
        elif t > self.cfg["stop_idle_confirm_sec"]:
            r["status"] = "not_confirmed"
            r["detail"] = "STOP NOT CONFIRMED. Use the physical E-stop."

    # ------------------------------------------------------------------ set-start flow (§5.3)
    def _flow_confirm(self, operator: str, args: Dict[str, Any], state: Dict[str, Any], now: float) -> Dict[str, Any]:
        f = self.flow
        choice = args.get("choice")
        if f.step == "where":
            f.choose_push(choice == "push")
            f.advance(now, {"choice": "push by hand" if f.push else "zero where it stands", "by": operator})
            return {"status": "confirmed", "detail": "pushing by hand first" if f.push else "zeroing where the robot stands"}
        if f.step == "estop":
            self.tracker.mem.estop_declaration = {"at": time.time(), "by": operator, "note": "declared, unverifiable"}
            f.advance(now, {"declared_by": operator, "at": time.time()})
            return {"status": "confirmed", "detail": "E-stop declared pressed (unverified)"}
        if f.step == "push":
            f.advance(now, {"declared_by": operator})
            return {"status": "confirmed", "detail": "robot declared at the new start spot"}
        if f.step == "release":
            f.released_at = now
            return {"status": "confirmed", "detail": "watching the base and arm"}
        return {"status": "rejected", "detail": "this step advances on its own when the console observes it"}

    def _flow_tick(self, state: Dict[str, Any], now: float) -> Optional[Dict[str, Any]]:
        f = self.flow
        if f is None:
            return None
        mem = self.tracker.mem
        j = state["joints"]["values"]
        t = now - f.t_step
        waiting, opts = "", []
        at_zero = j is not None and abs(j["base_x"]) < 0.001 and abs(j["base_y"]) < 0.001 and abs(j["base_theta"]) < 0.1
        if f.status == "active":
            if f.step == "precheck":
                ex = state["executor"]
                if not ex["fresh"] or ex["busy"] is False:
                    f.advance(now, {"executor": "idle" if ex["fresh"] else "not reporting"})
                else:
                    waiting = "The executor is busy. Press STOP and wait for it to go idle."
            if f.step == "where":
                pose = f"x {j['base_x']:+.2f} m, y {j['base_y']:+.2f} m, θ {j['base_theta']:+.1f}°" if j else "unknown"
                waiting = (f"Is the robot standing where start should be? It's at {pose} from the current origin. "
                           + ("Zeroing uses the ZERO command: no reset, the arm doesn't move." if f.method == "zero" else
                              "No ZERO command on this firmware: zeroing resets the Arduino, and the arm re-homes upright."))
                opts = [{"label": "Zero here", "choice": "here", "primary": True},
                        {"label": "I need to push it by hand", "choice": "push"}]
            elif f.step == "estop":
                waiting = ("Press the physical E-stop before pushing: with motors powered the base drives back to "
                           "its held target. The console can't see the E-stop; you declare it.")
                opts = [{"label": "I pressed the E-stop", "choice": "ok", "primary": True}]
            elif f.step == "push":
                waiting = "Push the robot to where start should be (the tape mark)."
                opts = [{"label": "Robot is on the start spot", "choice": "ok", "primary": True}]
            elif f.step == "zero":
                if not f.sent:
                    f.sent = True
                    f.notes["zero"] = {"old_boot": mem.session["id"], "method": f.method}
                    if f.method == "zero":
                        mem.zero_expected = True
                        res = self.io.zero()
                        if res is None:
                            f.method = "reset"
                            f.sent = False
                            mem.zero_expected = False
                            waiting = "ZERO isn't supported by this firmware; falling back to an Arduino reset."
                        elif res is False:
                            f.status, f.error = "error", "The Arduino refused ZERO (busy or not ready). Wait for idle and retry."
                            mem.zero_expected = False
                    if f.method == "reset" and f.sent:
                        mem.rezero_expected = True
                        self.io.publish_stop()
                        if not self.io.request_reset():
                            f.status, f.error = "error", ("The console can't reset the Arduino from here: restart "
                                                          "mac_serial_server.sh on the Mac (or replug USB), then retry.")
                    self.audit.write({"operator": f.by, "action": f"flow_set_start.zero.{f.method}", "args": {},
                                      "state_before": state["robot"]["state"], "result": {"status": "sent"}})
                if f.status == "active" and f.sent:
                    waiting = ("Sending ZERO: waiting for the base to read (0, 0, 0)." if f.method == "zero" else
                               "Resetting the Arduino: waiting for a new firmware boot with fresh joints at (0, 0, 0). "
                               "Stand clear: the arm re-homes upright if motors are powered.")
                    rebooted = f.method == "zero" or mem.session["id"] != f.notes["zero"]["old_boot"]
                    if rebooted and at_zero and state["joints"]["fresh"]:
                        f.advance(now, {"method": f.method, "boot_id": mem.session["id"]})
                    # reset: unit restart (~5-8 s) + Mega boot (~11 s observed on direct USB), with margin
                    elif t > (5.0 if f.method == "zero" else 45.0):
                        f.status, f.error = "error", ("No (0, 0, 0) reading after ZERO." if f.method == "zero" else
                                                      "No Arduino reset observed within 45 s. Check the serial bridge / USB.")
            if f.step == "crosscheck":
                waiting = "Watching the arm for 5 s: the firmware commands it upright at boot."
                if mem.session["upright_reached"] and t > 0.5:
                    f.crosscheck = "contradicted"
                    f.notes["crosscheck"] = {"result": "Motors are powered. The E-stop was NOT pressed."}
                    self.audit.write({"operator": f.by, "action": "flow_set_start.crosscheck",
                                      "result": {"status": "contradicted"}, "state_before": state["robot"]["state"]})
                    mem.estop_declaration = None
                    f.advance(now)
                elif t >= 5.0:
                    f.crosscheck = "consistent"
                    f.notes["crosscheck"] = {"result": "Arm stayed folded: consistent with the E-stop pressed."}
                    f.advance(now)
            if f.step == "release":
                if f.crosscheck == "contradicted":
                    f.released_at = f.released_at or now
                if f.released_at is None:
                    waiting = "Stand clear. Release the E-stop. The arm returns upright; the base should stay put."
                    opts = [{"label": "I released the E-stop", "choice": "ok", "primary": True}]
                else:
                    tr = now - f.released_at
                    waiting = f"Watching the base hold (0, 0, 0) and the arm rise to upright ({max(0, 10 - tr):.0f} s)."
                    base_ok = j is not None and abs(j["base_x"]) < 0.01 and abs(j["base_y"]) < 0.01
                    if j is not None and not base_ok:
                        f.status, f.error = "error", "The base moved more than 1 cm after release. Press STOP, then retry."
                    elif j is not None and abs(j["joint_elbow"]) < 3 and abs(j["joint_shoulder"]) < 3 and state["joints"]["arm_still"]:
                        mem.estop_declaration = None
                        f.advance(now, {"released_at": time.time()})
                    elif tr > 10.0:
                        f.status, f.error = "error", ("The arm didn't rise within 10 s. Motor power suspect: check the "
                                                      "E-stop, the arm power switch and the battery clips.")
            if f.step == "verify":
                waiting = "Verifying base_x, base_y, base_theta read ≈ 0 with fresh joints."
                if at_zero and state["joints"]["fresh"] and t > 0.5:
                    self.tracker.confirm_origin(f.by, f"set start here ({'ZERO' if f.method == 'zero' else 'reset'})")
                    f.advance(now, {"origin_at": time.time()})
                elif t > 5.0:
                    f.status, f.error = "error", "Base doesn't read (0, 0, 0). Retry the zero step."
        elif f.status == "error":
            waiting = f.error or ""
        return f.view(now, waiting, opts)

    # ------------------------------------------------------------------ periodic
    def tick(self, state: Dict[str, Any], now: float) -> Dict[str, Any]:
        """Verify tracked actions, STOP and the flow; auto-reject without a controller (§4.4)."""
        for t in self.tracked:
            if t["status"] != "sent":
                continue
            res = t["_check"](state)
            if res:
                t["status"], t["detail"] = res
            elif now - t["_t0"] > t["_timeout"]:
                t["status"] = "not_observed"
                t["detail"] = f"{t['detail']}: effect not observed within {t['_timeout']:.0f} s"
        self._verify_stop(state, now)
        ap = state["approval"]
        if ap and not self.lease.valid(now):
            self._no_controller_since = self._no_controller_since or now
            if now - self._no_controller_since >= self.cfg["approval_no_controller_reject_sec"]:
                self.io.send_approval(ap["plan_id"], False, "console")
                self.audit.write({"operator": "console", "action": "auto_reject", "args": {"plan_id": ap["plan_id"]},
                                  "state_before": state["robot"]["state"],
                                  "result": {"status": "sent", "detail": "no controlling client for 5 s"}})
                self.logs.add("console", "WARN", f"Auto-reject plan #{ap['plan_id']}: no controlling client for 5 s")
                self._no_controller_since = None
        else:
            self._no_controller_since = None
        return {"flow": self._flow_tick(state, now)}

    def tracked_view(self) -> List[Dict[str, Any]]:
        return [{k: v for k, v in t.items() if not k.startswith("_")} for t in self.tracked]

    def stop_view(self) -> Optional[Dict[str, Any]]:
        if not self.stop_result:
            return None
        return {k: v for k, v in self.stop_result.items() if not k.startswith("_")}
