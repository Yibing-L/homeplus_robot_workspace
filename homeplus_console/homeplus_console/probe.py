"""Is a component already running, whoever started it? (Hardware finding 2026-10-18: the console started a
second arduino_reader on a pty an external launch already held; two readers split every serial line.)

Evidence comes from the process table and open file descriptors (/proc, Linux), so it covers processes
started from tmux, other launch files or by hand, not only the console's own systemd units.
"""
from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Dict, List, Optional


def _cmdline(pid: str) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _pids() -> List[str]:
    try:
        return [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return []  # not Linux (e.g. the Mac running the sim): no evidence either way


def device_holders(path: str) -> List[Dict[str, str]]:
    """Processes with `path` (after resolving symlinks, e.g. ~/ttyArduino → /dev/pts/N) open."""
    try:
        target = os.path.realpath(os.path.expanduser(path))
    except OSError:
        return []
    out = []
    me = str(os.getpid())
    for pid in _pids():
        if pid == me:
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(f"{fd_dir}/{fd}") == target:
                    out.append({"pid": pid, "cmd": _cmdline(pid)[:160]})
                    break
            except OSError:
                continue
    return out


def processes_matching(patterns: List[str]) -> List[Dict[str, str]]:
    regs = [re.compile(p) for p in patterns]
    me = str(os.getpid())
    out = []
    for pid in _pids():
        if pid == me:
            continue
        cmd = _cmdline(pid)
        if cmd and any(r.search(cmd) for r in regs) and _same_ros_domain(pid):
            out.append({"pid": pid, "cmd": cmd[:160]})
    return out


def _ros_domain(environ: bytes) -> str:
    for item in environ.split(b"\0"):
        if item.startswith(b"ROS_DOMAIN_ID="):
            return item.split(b"=", 1)[1].decode(errors="replace").strip() or "0"
    return "0"


def _same_ros_domain(pid: str) -> bool:
    """A process on another ROS_DOMAIN_ID (e.g. a test bridge on a fake serial pty)
    cannot reach this robot's topics, so it is not a duplicate (2026-11-13)."""
    try:
        theirs = _ros_domain(Path(f"/proc/{pid}/environ").read_bytes())
    except OSError:
        return True  # unreadable: keep the conservative answer
    return theirs == (os.environ.get("ROS_DOMAIN_ID", "0").strip() or "0")


def cgroup_unit(pid: str) -> Optional[str]:
    """The systemd unit a process belongs to (e.g. 'hp-bridge.service'), if any."""
    try:
        for line in Path(f"/proc/{pid}/cgroup").read_text().splitlines():
            # innermost unit: user units sit under user@<uid>.service, e.g. .../user@1000.service/app.slice/hp-tf.service
            units = re.findall(r"/([\w@.\-]+\.service)", line)
            if units:
                return units[-1]
    except OSError:
        pass
    return None
