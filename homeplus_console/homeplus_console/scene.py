"""MoveIt planning scene for the 3D view: what the planner sees (OctoMap, world and attached objects).

Pure Python + numpy (no ROS, no octomap package), so the decoder is unit-tested and the sim can use it.

The OctoMap arrives as octomap_msgs/Octomap: an "OcTree" of depth 16 serialised in one of two streams.
  - full (binary=false; MoveIt's /get_planning_scene sends this): per node, depth first, a float32
    log-odds then one byte with a bit per allocated child (bit i = child i). A node without children
    is a leaf; it is occupied when log-odds >= 0 (OctoMap's default threshold, p = 0.5).
  - binary (binary=true): per inner node two bytes, 2 bits per child (children 0-3, then 4-7):
    00 none, 01 free leaf, 10 occupied leaf, 11 inner node; inner children follow depth first.
    (bit 2i set alone = free, bit 2i+1 set alone = occupied, both = inner.)
Child index i: bit 0 = +x half, bit 1 = +y half, bit 2 = +z half. The root covers keys [0, 65536) on
each axis and key 32768 is the coordinate origin, so a node at depth d with min-corner key m spans
2^(16-d) keys and its centre is (m - 32768 + 2^(16-d)/2) * resolution in the octree frame.

Voxels go to the browser as int16 rows (x, y, z, level): x/y/z = min-corner key - 32768, size =
resolution * 2^level. The octree frame is placed in the world by OctomapWithPose.origin.
"""
from __future__ import annotations

import base64
import hashlib
import math
import re
import struct
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

TREE_DEPTH = 16
KEY_ORIGIN = 1 << (TREE_DEPTH - 1)  # 32768
MAX_VOXELS = 100_000
FETCH_PERIOD = 1.0
STALE_SEC = 10.0
COLLISION_KEEP_SEC = 600.0
_F32 = struct.Struct("<f")

# "Collision overlay: <a> / <b> at <frame> [x, y, z]" (gesture_motion_executor._publish_collision_debug)
_COLLISION = re.compile(r"Collision overlay: (?P<a>\S+) / (?P<b>\S+) at (?P<frame>\S+) "
                        r"\[(?P<x>-?[\d.]+), (?P<y>-?[\d.]+), (?P<z>-?[\d.]+)\]")


class OctreeError(ValueError):
    pass


# ---------------------------------------------------------------------------------------------- decode
def decode_octree(data: bytes, binary: bool, occupancy_threshold: float = 0.0,
                  max_voxels: int = MAX_VOXELS) -> Tuple[np.ndarray, bool]:
    """Occupied leaves of an OcTree stream → (int16 array N×4 of x, y, z, level; truncated)."""
    out: List[Tuple[int, int, int, int]] = []
    if not data:
        return np.zeros((0, 4), np.int16), False
    try:
        if binary:
            truncated = _decode_binary(data, out, max_voxels)
        else:
            truncated = _decode_full(data, out, occupancy_threshold, max_voxels)
    except (struct.error, IndexError) as exc:
        raise OctreeError(f"octree stream ends early ({len(data)} bytes)") from exc
    arr = np.array(out, dtype=np.int32).reshape(-1, 4)
    arr[:, :3] -= KEY_ORIGIN
    return arr.astype(np.int16), truncated


def _decode_full(data: bytes, out: list, thr: float, cap: int) -> bool:
    pos = 0
    stack = [(0, 0, 0, 0)]  # (depth, min-corner key x, y, z)
    while stack:
        d, x, y, z = stack.pop()
        logodds = _F32.unpack_from(data, pos)[0]
        mask = data[pos + 4]
        pos += 5
        if not mask:
            if logodds >= thr:
                if len(out) >= cap:
                    return True
                out.append((x, y, z, TREE_DEPTH - d))
            continue
        if d >= TREE_DEPTH:
            raise OctreeError("node below depth 16 has children")
        half = 1 << (TREE_DEPTH - d - 1)
        for i in range(7, -1, -1):  # pushed in reverse: children are stored in order 0..7
            if mask >> i & 1:
                stack.append((d + 1, x + (half if i & 1 else 0), y + (half if i & 2 else 0), z + (half if i & 4 else 0)))
    if pos != len(data):
        raise OctreeError(f"{len(data) - pos} trailing bytes after the octree")
    return False


def _decode_binary(data: bytes, out: list, cap: int) -> bool:
    pos = 0
    stack = [(0, 0, 0, 0)]  # inner nodes still to read
    while stack:
        d, x, y, z = stack.pop()
        if d >= TREE_DEPTH:
            raise OctreeError("inner node at depth 16")
        bits = data[pos] | data[pos + 1] << 8
        pos += 2
        half = 1 << (TREE_DEPTH - d - 1)
        inner = []
        for i in range(8):
            code = bits >> (2 * i) & 3
            if code < 2:  # 0 = no child, 1 = free leaf
                continue
            cx, cy, cz = x + (half if i & 1 else 0), y + (half if i & 2 else 0), z + (half if i & 4 else 0)
            if code == 2:
                if len(out) >= cap:
                    return True
                out.append((cx, cy, cz, TREE_DEPTH - d - 1))
            else:
                inner.append((d + 1, cx, cy, cz))
        stack.extend(reversed(inner))
    if pos != len(data):
        raise OctreeError(f"{len(data) - pos} trailing bytes after the octree")
    return False


def voxel_centres(vox: np.ndarray, resolution: float) -> np.ndarray:
    """N×4 int16 rows → N×3 centres (m, octree frame) and sizes; for tests and the sim."""
    v = vox.astype(np.float64)
    size = np.exp2(v[:, 3])
    return np.column_stack([(v[:, :3] + size[:, None] / 2) * resolution, size * resolution])


# ---------------------------------------------------------------------------------------------- encode
def _leaf_keys(points: Iterable[Sequence[float]], resolution: float) -> set:
    keys = set()
    for p in points:
        keys.add(tuple(int(math.floor(c / resolution)) + KEY_ORIGIN for c in p))
    return keys


def encode_octree(occupied: Iterable[Sequence[float]], resolution: float, binary: bool,
                  free: Iterable[Sequence[float]] = ()) -> bytes:
    """Unpruned OcTree stream with the given points as depth-16 occupied (and free) leaves.

    Writes inner nodes the way OctoMap does (full: max child log-odds). Used by the sim and the tests;
    real trees from MoveIt are pruned, which the decoder handles through the per-voxel level.
    """
    leaves = {k: False for k in _leaf_keys(free, resolution)}
    leaves.update({k: True for k in _leaf_keys(occupied, resolution)})
    if not leaves:
        return b""
    buf = bytearray()

    def children(d: int, x: int, y: int, z: int, keys: List[Tuple[int, int, int]]):
        half = 1 << (TREE_DEPTH - d - 1)
        groups: Dict[int, List[Tuple[int, int, int]]] = {}
        for k in keys:
            groups.setdefault((k[0] - x >= half) | (k[1] - y >= half) << 1 | (k[2] - z >= half) << 2, []).append(k)
        for i in sorted(groups):
            yield i, x + (half if i & 1 else 0), y + (half if i & 2 else 0), z + (half if i & 4 else 0), groups[i]

    def full(d: int, x: int, y: int, z: int, keys) -> float:
        if d == TREE_DEPTH:
            v = 2.0 if leaves[(x, y, z)] else -2.0
            buf.extend(_F32.pack(v) + b"\0")
            return v
        at = len(buf)
        buf.extend(b"\0\0\0\0\0")
        mask, best = 0, -2.0
        for i, cx, cy, cz, ck in children(d, x, y, z, keys):
            mask |= 1 << i
            best = max(best, full(d + 1, cx, cy, cz, ck))
        buf[at:at + 4] = _F32.pack(best)
        buf[at + 4] = mask
        return best

    def binary_(d: int, x: int, y: int, z: int, keys) -> None:
        kids = list(children(d, x, y, z, keys))
        bits = 0
        for i, cx, cy, cz, _ in kids:
            if d + 1 == TREE_DEPTH:
                bits |= (2 if leaves[(cx, cy, cz)] else 1) << (2 * i)
            else:
                bits |= 3 << (2 * i)
        buf.extend((bits & 0xFF, bits >> 8))
        for i, cx, cy, cz, ck in kids:
            if d + 1 < TREE_DEPTH:
                binary_(d + 1, cx, cy, cz, ck)

    (binary_ if binary else full)(0, 0, 0, 0, list(leaves))
    return bytes(buf)


# ---------------------------------------------------------------------------------------------- misc
def parse_collision(text: str) -> Optional[Dict[str, Any]]:
    m = _COLLISION.search(text)
    if not m:
        return None
    return {"a": m["a"], "b": m["b"], "frame": m["frame"], "p": [float(m["x"]), float(m["y"]), float(m["z"])]}


def quat_mul(a: Sequence[float], b: Sequence[float]) -> List[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def quat_rotate(q: Sequence[float], v: Sequence[float]) -> List[float]:
    x, y, z, w = q
    r = quat_mul(quat_mul(q, [v[0], v[1], v[2], 0.0]), [-x, -y, -z, w])
    return r[:3]


def compose(a: Dict[str, List[float]], b: Dict[str, List[float]]) -> Dict[str, List[float]]:
    """Pose a ∘ b, poses as {"p": [x, y, z], "q": [x, y, z, w]}."""
    p = quat_rotate(a["q"], b["p"])
    return {"p": [a["p"][i] + p[i] for i in range(3)], "q": quat_mul(a["q"], b["q"])}


IDENTITY = {"p": [0.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}


# ---------------------------------------------------------------------------------------------- worker
class SceneWorker:
    """Polls the planning scene at ~1 Hz on its own thread while a browser is connected.

    fetch() is RobotIO.fetch_scene: blocking, returns
      {"frame": "world", "octomap": {"id", "binary", "resolution", "data": bytes, "origin": pose} | None,
       "objects": [{"id", "kind", "dims", "p", "q"}], "attached": [{"id", "link", "kind", "dims", "p", "q"}]}
    with objects in the world frame and attached ones relative to their link; or raises / returns
    {"error": str}. The derive loop only reads the latest result, so a slow move_group never blocks it.
    """

    def __init__(self, fetch: Optional[Callable[[], Optional[Dict[str, Any]]]], logs: Any,
                 clock: Callable[[], float] = time.monotonic, period: float = FETCH_PERIOD):
        self.fetch, self.logs, self.clock, self.period = fetch, logs, clock, period
        self.clients = 0
        self._lock = threading.Lock()
        self._scene: Optional[Dict[str, Any]] = None
        self._at: Optional[float] = None
        self._error: Optional[str] = None
        self._oct_hash: Optional[bytes] = None
        self._vox: Dict[str, Any] = {"seq": 0, "count": 0, "resolution": None, "origin": IDENTITY, "data": ""}
        self._oct_meta: Optional[Dict[str, Any]] = None
        self._collision: Optional[Dict[str, Any]] = None
        self._log_seq = 0
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._loop, name="scene", daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.scan_logs()
            if self.clients > 0 and self.fetch is not None:
                self.poll()
            self._stop.wait(max(0.2, self.period - (time.monotonic() - t0)))

    def scan_logs(self) -> None:
        lines = self.logs.since(self._log_seq, limit=2000)
        for line in lines:
            self._log_seq = max(self._log_seq, line["seq"])
            if "Collision overlay:" in line["text"] and (c := parse_collision(line["text"])):
                c["t_wall"] = line["t_wall"]
                c["mono"] = self.clock() - max(0.0, time.time() - line["t_wall"])
                with self._lock:
                    self._collision = c

    def poll(self) -> None:
        try:
            res = self.fetch() if self.fetch else None
        except Exception as exc:  # noqa: BLE001 (rclpy / decode failures are reported, never fatal)
            res = {"error": f"{type(exc).__name__}: {exc}"}
        if res is None:
            return
        if res.get("error"):
            with self._lock:
                self._error = res["error"]
            return
        oct_msg = res.get("octomap")
        meta = self._oct_meta
        if oct_msg is None:
            meta, vox = None, {"seq": self._vox["seq"] + (1 if self._vox["count"] else 0), "count": 0,
                               "resolution": None, "origin": IDENTITY, "data": ""}
        else:
            data = bytes(oct_msg.get("data") or b"")
            h = hashlib.blake2b(data + repr((oct_msg.get("binary"), oct_msg.get("resolution"),
                                             oct_msg.get("origin"))).encode(), digest_size=16).digest()
            vox = None
            if h != self._oct_hash or meta is None:
                meta, vox = self._decode(oct_msg, data)
                self._oct_hash = h if vox is not None else None
        with self._lock:
            if vox is not None:
                self._vox = vox
            self._oct_meta = meta
            self._scene = res
            self._at = self.clock()
            self._error = (meta or {}).get("error")

    def _decode(self, oct_msg: Dict[str, Any], data: bytes) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
        res = float(oct_msg.get("resolution") or 0.0)
        oid = oct_msg.get("id", "")
        base = {"id": oid, "format": "binary" if oct_msg.get("binary") else "full", "bytes": len(data),
                "resolution": res, "count": 0, "truncated": False, "decode_ms": None, "error": None}
        if oid != "OcTree" or res <= 0:
            return {**base, "error": f"octomap type {oid!r} not supported (OcTree only)"}, None
        t0 = time.perf_counter()
        try:
            vox, truncated = decode_octree(data, bool(oct_msg.get("binary")))
        except OctreeError as exc:
            return {**base, "error": f"octomap decode failed: {exc}"}, None
        ms = round((time.perf_counter() - t0) * 1000, 1)
        seq = self._vox["seq"] + 1
        meta = {**base, "count": int(len(vox)), "truncated": truncated, "decode_ms": ms, "seq": seq}
        return meta, {"seq": seq, "count": int(len(vox)), "resolution": res,
                      "origin": oct_msg.get("origin") or IDENTITY,
                      "data": base64.b64encode(vox.astype("<i2").tobytes()).decode("ascii")}

    def voxels(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._vox)

    def view(self, now: float) -> Dict[str, Any]:
        with self._lock:
            sc, at, err, meta, col, vox_seq = self._scene, self._at, self._error, self._oct_meta, self._collision, self._vox["seq"]
        age = None if at is None else max(0.0, now - at)
        out: Dict[str, Any] = {"polling": self.clients > 0 and self.fetch is not None, "age": age,
                               "stale": age is None or age > STALE_SEC, "error": err, "frame": None,
                               "octomap": None, "objects": [], "attached": [], "skipped": [], "collision": None}
        if sc is not None:
            out["frame"] = sc.get("frame", "world")
            out["objects"] = sc.get("objects") or []
            out["attached"] = sc.get("attached") or []
            out["skipped"] = sc.get("skipped") or []
            if meta is not None:
                out["octomap"] = {**meta, "seq": vox_seq}
        if col is not None:
            c_age = max(0.0, now - col["mono"])
            if c_age <= COLLISION_KEEP_SEC:
                out["collision"] = {k: v for k, v in col.items() if k != "mono"} | {"age": c_age}
        return out
