"""/rosout log buffer, the "important line" filter, and the status-string fallback parser.

The important-line regexes are the ones listed in UI_PLAN §4.3, taken from the executor
and bridge log strings on branch ev-closed-loop-pickup-fixes.
"""
from __future__ import annotations

import re
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from typing import Deque, Dict, List, Optional

IMPORTANT = [re.compile(p) for p in (
    r"Step \d+/\d+",
    r"step \d+/\d+",
    r"measured endpoint verified",
    r"DONE without a settled measured endpoint",
    r"IK preflight .*: \d+ valid, rejected=",
    r"ABORTED at",
    r"Invalid state for .*: contacts=",
    r"no valid IK goal",
    r"Cartesian approach incomplete",
    r"Arduino execution error: ERROR",
    r"MoveIt failed for",
    r"waiting for approval",
    r"Arduino serial",
    r"Connected .* waiting for firmware state feedback",
    r"No fresh firmware state feedback",
    r"STOP",
    r"FAILED",
    r"Starting sequence",
    r"Sequence .* complete",
)]

LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


@dataclass
class LogLine:
    seq: int
    t_wall: float
    node: str
    level: str
    text: str
    important: bool
    source: str = "rosout"   # "rosout", "unit:<name>", "outside console", "ambiguous: …", "console"


class LogBuffer:
    def __init__(self, maxlen: int = 5000) -> None:
        self._lock = threading.Lock()
        self._lines: Deque[LogLine] = deque(maxlen=maxlen)
        self._seq = 0

    def add(self, node: str, level: str, text: str, t_wall: Optional[float] = None, source: Optional[str] = None) -> LogLine:
        important = level in ("WARN", "ERROR", "FATAL") or any(p.search(text) for p in IMPORTANT)
        if source is None:
            source = "console" if node in ("console", "sim") else "rosout"
        with self._lock:
            self._seq += 1
            line = LogLine(self._seq, t_wall or time.time(), node, level, text, important, source)
            self._lines.append(line)
            return line

    def retag(self, node: str, since_wall: float, source: str) -> int:
        """Re-attribute recent lines once it turns out two processes shared the node name."""
        n = 0
        with self._lock:
            for line in self._lines:
                if line.node == node and line.t_wall >= since_wall and not line.source.startswith("ambiguous"):
                    line.source = source
                    n += 1
        return n

    def since(self, seq: int = 0, important_only: bool = False, limit: int = 500) -> List[Dict]:
        with self._lock:
            lines = [l for l in self._lines if l.seq > seq and (l.important or not important_only)]
        return [asdict(l) for l in lines[-limit:]]

    def last_matching(self, node: str, pattern: re.Pattern, within_sec: float) -> Optional[LogLine]:
        cutoff = time.time() - within_sec
        with self._lock:
            for line in reversed(self._lines):
                if line.t_wall < cutoff:
                    return None
                if line.node == node and pattern.search(line.text):
                    return line
        return None


# Degraded-mode fallback (UI_PLAN §4.3): derive a coarse executor state from
# /gesture_control/executor_status strings when executor_state is not published.
_STEP = re.compile(r"\[(?P<seq>[\w_]+)\] step (?P<i>\d+)/(?P<n>\d+): (?P<name>[\w_]+)")
_WAIT = re.compile(r"Plan for (?P<name>[\w_]+) is waiting for approval")
_ABORT = re.compile(r"Sequence (?P<seq>[\w_]+) ABORTED at (?P<step>[\w_]+): (?P<reason>.*)")
_FAIL = re.compile(r"Command (?P<cmd>[\w_]+) FAILED: (?P<reason>.*)")
_HOLDING = re.compile(r"^Holding cup: (?P<yes>yes|no) ")


def parse_status_text(text: str) -> Optional[Dict]:
    m = _STEP.search(text)
    if m:
        return {"kind": "step", "sequence": m["seq"], "step_index": int(m["i"]),
                "step_count": int(m["n"]), "step_name": m["name"]}
    m = _WAIT.search(text)
    if m:
        return {"kind": "awaiting_approval", "step_name": m["name"]}
    m = _ABORT.search(text)
    if m:
        return {"kind": "aborted", "sequence": m["seq"], "step": m["step"], "reason": m["reason"]}
    m = _FAIL.search(text)
    if m:
        return {"kind": "failed", "command": m["cmd"], "reason": m["reason"]}
    m = _HOLDING.search(text)
    if m:
        return {"kind": "holding", "holding": m["yes"] == "yes"}
    return None
