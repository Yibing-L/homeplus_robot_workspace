"""systemd --user transient units, one per component (UI_PLAN §5.1).

Start is idempotent; stop kills the whole control group so a `ros2 launch` can't leave orphans;
status comes from `systemctl --user show`. Polled on its own thread so the derive loop never blocks
on a subprocess.
"""
from __future__ import annotations

import shlex
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

from . import probe
from .log_filter import LogBuffer

SHOW = "ActiveState,SubState,MainPID,ExecMainStartTimestampMonotonic,NRestarts,Result"

# How to recognise each component no matter who started it. A profile can extend these per unit with
# `probe: {nodes: [...], procs: [...], device: path}`.
DEFAULT_PROBES: Dict[str, Dict[str, Any]] = {
    "hp-bridge": {"nodes": ["arduino_bridge"], "procs": [r"arduino_reader\.py"]},
    "hp-vision": {"nodes": ["molmopoint"], "procs": [r"vision_pipeline\.launch\.py", r"grounding_dino_node"]},
    "hp-tf": {"nodes": ["robot_state_publisher"], "procs": [r"state_publisher\.launch\.py"]},
    "hp-moveit": {"nodes": ["move_group", "gesture_motion_executor"],
                  "procs": [r"gesture_control\.launch\.py", r"/move_group(\s|$)"]},
    "hp-camera-rx": {"nodes": [], "procs": [r"d405_vm_receiver\.py"]},
    "hp-serial-vm": {"nodes": [], "procs": [r"vm_serial_client\.sh", r"socat .*ttyArduino"]},
}


class SystemdSupervisor:
    def __init__(self, units: Dict[str, Dict[str, Any]], logs: LogBuffer, ws_setup: str = ""):
        self.spec = units
        self.units: List[str] = list(units)
        self.probes: Dict[str, Dict[str, Any]] = {}
        for u in set(units) | set(DEFAULT_PROBES):
            pr = dict(DEFAULT_PROBES.get(u, {"nodes": [], "procs": []}))
            pr.update((units.get(u) or {}).get("probe") or {})
            self.probes[u] = pr
        self.logs = logs
        self._lock = threading.Lock()
        self._status: Dict[str, Dict[str, Any]] = {u: self._empty() for u in self.units}

    @staticmethod
    def _empty() -> Dict[str, Any]:
        return {"active_state": "inactive", "sub_state": "dead", "pid": None, "start_mono": None,
                "restarts": 0, "result": "success", "uptime_s": None}

    def _run(self, *args: str, timeout: float = 10.0) -> subprocess.CompletedProcess:
        return subprocess.run(list(args), capture_output=True, text=True, timeout=timeout)

    def _poll_once(self) -> None:
        now_us = time.monotonic() * 1e6
        for u in self.units:
            try:
                r = self._run("systemctl", "--user", "show", u, "-p", SHOW, timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                continue
            kv = dict(line.split("=", 1) for line in r.stdout.splitlines() if "=" in line)
            start = int(kv.get("ExecMainStartTimestampMonotonic", "0") or 0)
            st = {"active_state": kv.get("ActiveState", "inactive"), "sub_state": kv.get("SubState", "dead"),
                  "pid": int(kv.get("MainPID", "0") or 0) or None, "start_mono": start / 1e6 if start else None,
                  "restarts": int(kv.get("NRestarts", "0") or 0), "result": kv.get("Result", "success"),
                  "uptime_s": (now_us - start) / 1e6 if start and kv.get("ActiveState") == "active" else None}
            with self._lock:
                self._status[u] = st

    def start_polling(self, period: float = 1.0) -> None:
        def loop():
            while True:
                self._poll_once()
                time.sleep(period)
        threading.Thread(target=loop, name="supervisor", daemon=True).start()

    def status(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return {u: dict(s) for u, s in self._status.items()}

    def is_active(self, unit: str) -> bool:
        return self.status().get(unit, {}).get("active_state") == "active"

    def external(self, unit: str, graph_nodes: Dict[str, int]) -> List[str]:
        """Evidence that `unit`'s component is running outside its console unit. Empty = none found.
        Processes inside the unit's own cgroup are not evidence; ROS node names only count while the
        unit itself is down (when it's up, its own node has that name)."""
        pr = self.probes.get(unit, {})
        own = f"{unit}.service"
        active = self.is_active(unit)
        out: List[str] = []
        seen = set()
        if pr.get("device"):
            holders = [h for h in probe.device_holders(pr["device"]) if probe.cgroup_unit(h["pid"]) != own]
            if holders:
                out.append(f"{pr['device']} is open by " + "; ".join(f"pid {h['pid']} ({h['cmd']})" for h in holders))
                seen = {h["pid"] for h in holders}
        for h in probe.processes_matching(pr.get("procs", [])):
            if h["pid"] not in seen and probe.cgroup_unit(h["pid"]) != own:
                out.append(f"pid {h['pid']} is running {h['cmd']}")
        if not active:
            for n in pr.get("nodes", []):
                if graph_nodes.get(n):
                    out.append(f"node /{n} is on the ROS graph")
        return out

    def node_unit(self, node: str) -> Optional[str]:
        for u, pr in self.probes.items():
            if node in pr.get("nodes", []):
                return u
        return None

    def start(self, unit: str) -> Dict[str, Any]:
        st = self.status().get(unit, {})
        if st.get("active_state") in ("active", "activating"):
            return {"result": "already_running", "pid": st.get("pid")}
        spec = self.spec[unit]
        env = [f"-E{k}={v}" for k, v in (spec.get("env") or {}).items()]
        cmd = spec["cmd"]
        try:
            r = self._run("systemd-run", "--user", f"--unit={unit}", "--collect", "--property=KillMode=control-group",
                          "--property=TimeoutStopSec=10", *env, "bash", "-lc", cmd)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"result": f"failed: {exc}"}
        self.logs.add("console", "INFO" if r.returncode == 0 else "ERROR",
                      f"systemd-run {unit}: {(r.stderr or r.stdout).strip()[:200] or 'started'}")
        return {"result": "started" if r.returncode == 0 else f"failed: {r.stderr.strip()[:200]}"}

    def stop(self, unit: str) -> Dict[str, Any]:
        try:
            r = self._run("systemctl", "--user", "stop", unit, timeout=15)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"result": f"failed: {exc}"}
        return {"result": "stopped" if r.returncode == 0 else f"failed: {r.stderr.strip()[:200]}"}

    def restart(self, unit: str) -> Dict[str, Any]:
        self.stop(unit)
        return self.start(unit)

    def journal(self, unit: str, lines: int = 200) -> List[str]:
        try:
            r = self._run("journalctl", "--user", "-u", unit, "-n", str(lines), "--no-pager", "-o", "short-iso", timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return []
        return r.stdout.splitlines()

    @staticmethod
    def quote(cmd: str) -> str:
        return shlex.quote(cmd)
