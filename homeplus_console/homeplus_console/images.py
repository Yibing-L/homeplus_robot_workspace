"""Latest-frame store for every camera feed, plus frozen-content detection.

Frames are kept as JPEG bytes and served per sequence number (/img/<feed>.jpg?seq=N).
The browser only fetches a frame when the snapshot says a newer one exists, so a slow
link drops frames instead of queueing them, and nothing here can block the robot.
"""
from __future__ import annotations

import hashlib
import io
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from PIL import Image

FROZEN_AFTER_SEC = 3.0


@dataclass
class Frame:
    seq: int
    jpeg: bytes
    width: int
    height: int
    recv_mono: float
    thumb_hash: str


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: Dict[str, Frame] = {}
        # feed -> (hash, first time this hash was seen)
        self._hash_since: Dict[str, Tuple[str, float]] = {}

    def put_image(self, feed: str, img: Image.Image, now: float, quality: int = 70) -> Frame:
        buf = io.BytesIO()
        img.convert("RGB").save(buf, format="JPEG", quality=quality)
        thumb = img.convert("L").resize((32, 32))
        h = hashlib.blake2b(thumb.tobytes(), digest_size=8).hexdigest()
        with self._lock:
            prev = self._frames.get(feed)
            seq = (prev.seq + 1) if prev else 1
            frame = Frame(seq=seq, jpeg=buf.getvalue(), width=img.width, height=img.height,
                          recv_mono=now, thumb_hash=h)
            self._frames[feed] = frame
            last = self._hash_since.get(feed)
            if last is None or last[0] != h:
                self._hash_since[feed] = (h, now)
            return frame

    def get(self, feed: str) -> Optional[Frame]:
        with self._lock:
            return self._frames.get(feed)

    def frozen_for(self, feed: str, now: float) -> float:
        """Seconds the thumbnail hash has been unchanged (0 if it changed on the last frame)."""
        with self._lock:
            frame = self._frames.get(feed)
            since = self._hash_since.get(feed)
        if frame is None or since is None:
            return 0.0
        # Measured up to the last received frame: a stopped feed is stale, not frozen.
        return max(0.0, frame.recv_mono - since[1])
