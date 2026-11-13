"""Demonstration recorder for teleop: one directory per episode, convertible to a LeRobot dataset.

The console has no pyarrow/ffmpeg, so it writes a plain, lossless-enough raw episode and
tools/teleop_to_lerobot.py turns a set of episodes into a LeRobot dataset (parquet + mp4) in an
environment that has `lerobot` installed.

    <data_dir>/teleop_episodes/<YYYYmmdd-HHMMSS>-<id>/
        meta.json        instruction, operator, fps, joint names, cameras, outcome, timing
        steps.jsonl      one line per sample (fps, default 10 Hz)
        images/<camera>/<index>.jpg     JPEG q90 at the camera's native size

steps.jsonl fields (SI units: m, rad):
    i, t (s since start), wall
    state      measured joints, JOINT_NAMES order (= observation.state)
    action     commanded absolute joint setpoint sent to the bridge, same order (null while not streaming)
    base_vel   commanded body velocity [vx m/s, vy m/s, wz rad/s] (an alternative base action)
    intent     the operator's input axes this tick
    images     {camera: relative path or null}, image_age {camera: s}

Images are encoded on a writer thread so the 25 Hz teleop loop never waits on JPEG encoding.
"""
from __future__ import annotations

import io as _io
import json
import queue
import shutil
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional

from .teleop import ORDER

CAMERAS = {"hand": "d405_color", "front": "d455_color"}


class Recorder:
    def __init__(self, root: Path, io, fps: float = 10.0, logs=None):
        self.root, self.io, self.fps, self.logs = root, io, fps, logs
        self.active = False
        self.dir: Optional[Path] = None
        self.meta: Dict[str, Any] = {}
        self.n = 0
        self.t0 = 0.0
        self.last = -1e9
        self.dropped = 0
        self.last_result: Optional[str] = None
        self._q: "queue.Queue" = queue.Queue(maxsize=200)
        self._steps = None
        threading.Thread(target=self._writer, name="recorder", daemon=True).start()

    def start(self, instruction: str, operator: str) -> str:
        if self.active:
            self.stop("aborted", "new recording started")
        eid = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:4]
        self.dir = self.root / eid
        for cam in CAMERAS:
            (self.dir / "images" / cam).mkdir(parents=True, exist_ok=True)
        self.meta = {"episode": eid, "instruction": instruction, "operator": operator, "fps": self.fps,
                     "joint_names": list(ORDER), "units": "SI (m, rad)", "cameras": dict(CAMERAS),
                     "action": "absolute joint setpoint streamed to the bridge (JOINT_NAMES order)",
                     "robot": "homeplus", "start_wall": time.time(), "outcome": None}
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2))
        self._steps = open(self.dir / "steps.jsonl", "a")
        self.n, self.t0, self.last, self.dropped = 0, time.monotonic(), -1e9, 0
        self.active = True
        if self.logs:
            self.logs.add("console", "INFO", f"Recording episode {eid}: {instruction!r}")
        return str(self.dir)

    def sample(self, now: float, measured, action, base_vel, intent) -> None:
        if not self.active or now - self.last < 1.0 / self.fps - 1e-3 or measured is None:
            return
        self.last = now
        i = self.n
        self.n += 1
        images, ages = {}, {}
        for cam, feed in CAMERAS.items():
            fr = self.io.raw_frame(feed)
            if fr is None:
                images[cam], ages[cam] = None, None
                continue
            t_img, img = fr
            rel = f"images/{cam}/{i:06d}.jpg"
            images[cam], ages[cam] = rel, round(now - t_img, 3)
            try:
                self._q.put_nowait((self.dir / rel, img))
            except queue.Full:
                self.dropped += 1
                images[cam] = None
        row = {"i": i, "t": round(now - self.t0, 4), "wall": time.time(),
               "state": [measured[n] for n in ORDER],
               "action": None if action is None else [action[n] for n in ORDER],
               "base_vel": None if base_vel is None else list(base_vel),
               "intent": {"fwd": intent.fwd, "strafe": intent.strafe, "turn": intent.turn, "pitch": intent.pitch,
                          "roll": intent.roll, "grip": intent.grip, "drag": intent.drag, "fast": intent.fast},
               "images": images, "image_age": ages}
        try:
            self._q.put_nowait(("row", row))
        except queue.Full:   # never block the teleop loop; the gap shows in steps.jsonl indices
            self.dropped += 1

    def stop(self, outcome: str, note: str = "") -> str:
        if not self.active:
            return "not recording"
        self.active = False
        self._q.join()
        if self._steps:
            self._steps.close()
            self._steps = None
        d = self.dir
        if outcome == "discard":
            shutil.rmtree(d, ignore_errors=True)
            self.last_result = f"episode discarded ({self.n} samples)"
        else:
            self.meta.update(outcome=outcome, note=note, end_wall=time.time(), samples=self.n,
                             duration_s=round(time.monotonic() - self.t0, 2), dropped_images=self.dropped)
            (d / "meta.json").write_text(json.dumps(self.meta, indent=2))
            self.last_result = f"saved {self.n} samples ({outcome}) to {d}"
        if self.logs:
            self.logs.add("console", "INFO", f"Recording stopped: {self.last_result}")
        return self.last_result

    def _writer(self) -> None:
        while True:
            item = self._q.get()
            try:
                if item[0] == "row":
                    if self._steps:
                        self._steps.write(json.dumps(item[1]) + "\n")
                        self._steps.flush()
                else:
                    path, img = item
                    if isinstance(img, (bytes, bytearray)):
                        path.write_bytes(img)
                    else:
                        buf = _io.BytesIO()
                        img.convert("RGB").save(buf, format="JPEG", quality=90)
                        path.write_bytes(buf.getvalue())
            except Exception as exc:  # noqa: BLE001
                if self.logs:
                    self.logs.add("console", "ERROR", f"recorder write failed: {exc!r}")
            finally:
                self._q.task_done()

    def view(self) -> Dict[str, Any]:
        return {"active": self.active, "episode": self.meta.get("episode") if self.active else None,
                "instruction": self.meta.get("instruction") if self.active else None,
                "samples": self.n if self.active else None,
                "seconds": round(time.monotonic() - self.t0, 1) if self.active else None,
                "dropped": self.dropped if self.active else None, "last_result": self.last_result,
                "root": str(self.root)}
