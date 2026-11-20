"""Signal store: every value the console knows, with its receipt time and source.

Age is always measured from receipt on the console's monotonic clock (UI_PLAN §2).
The clock is injectable so staleness thresholds can be tested with a frozen clock.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

Clock = Callable[[], float]

STRIP_WINDOW_SEC = 60.0
STRIP_BINS = 120  # 0.5 s per bin


@dataclass
class Signal:
    name: str
    source: str
    value: Any = None
    recv_mono: Optional[float] = None
    header_stamp: Optional[float] = None
    count: int = 0
    _recv_times: Deque[float] = field(default_factory=lambda: deque(maxlen=4096))

    def age(self, now: float) -> Optional[float]:
        if self.recv_mono is None:
            return None
        return max(0.0, now - self.recv_mono)

    def fresh(self, now: float, max_age: float) -> bool:
        age = self.age(now)
        return age is not None and age <= max_age

    def rate_hz(self, now: float, window: float = 10.0) -> float:
        n = sum(1 for t in self._recv_times if now - t <= window)
        return n / window

    def strip(self, now: float) -> str:
        """Receipt density over the last 60 s, oldest bin first: '0'..'9' per 0.5 s bin."""
        bins = [0] * STRIP_BINS
        width = STRIP_WINDOW_SEC / STRIP_BINS
        for t in self._recv_times:
            age = now - t
            if 0 <= age < STRIP_WINDOW_SEC:
                idx = STRIP_BINS - 1 - int(age / width)
                bins[idx] += 1
        return "".join(str(min(9, b)) for b in bins)


class SignalStore:
    """Thread-safe store. ROS callbacks (or the sim) write; the state machine reads snapshots."""

    def __init__(self, clock: Clock = time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._signals: Dict[str, Signal] = {}
        self._events: Deque[Tuple[int, str, Any, float]] = deque(maxlen=2000)
        self._event_seq = 0

    def update(self, name: str, value: Any, source: str, header_stamp: Optional[float] = None) -> None:
        now = self.clock()
        with self._lock:
            sig = self._signals.get(name)
            if sig is None:
                sig = self._signals[name] = Signal(name=name, source=source)
            sig.value = value
            sig.source = source
            sig.recv_mono = now
            sig.header_stamp = header_stamp
            sig.count += 1
            sig._recv_times.append(now)

    def push_event(self, name: str, value: Any, source: str) -> None:
        """Event topics (/arduino/ack, diagnostics cycles, poses): kept as a queue so none are lost
        between state-machine ticks, and also stored as a normal signal for freshness."""
        self.update(name, value, source)
        with self._lock:
            self._event_seq += 1
            self._events.append((self._event_seq, name, value, self.clock()))

    def events_since(self, seq: int) -> List[Tuple[int, str, Any, float]]:
        with self._lock:
            return [e for e in self._events if e[0] > seq]

    def get(self, name: str) -> Optional[Signal]:
        with self._lock:
            return self._signals.get(name)

    def snapshot(self) -> Dict[str, Signal]:
        """Shallow copy; Signal fields are only ever replaced, never mutated in place, except deque appends."""
        with self._lock:
            out = {}
            for k, s in self._signals.items():
                out[k] = Signal(
                    name=s.name, source=s.source, value=s.value, recv_mono=s.recv_mono,
                    header_stamp=s.header_stamp, count=s.count, _recv_times=deque(s._recv_times),
                )
            return out

    def forget(self, name: str) -> None:
        with self._lock:
            self._signals.pop(name, None)


def sig_view(sig: Optional[Signal], now: float, max_age: float, source: str = "") -> Dict[str, Any]:
    """Serializable three-valued view of a signal. Stale values move to last_value; value becomes None."""
    if sig is None:
        return {"value": None, "last_value": None, "age": None, "fresh": False, "source": source,
                "rate_hz": 0.0, "strip": "0" * STRIP_BINS}
    fresh = sig.fresh(now, max_age)
    return {
        "value": sig.value if fresh else None,
        "last_value": sig.value,
        "age": sig.age(now),
        "fresh": fresh,
        "source": sig.source,
        "rate_hz": round(sig.rate_hz(now), 2),
        "strip": sig.strip(now),
    }
