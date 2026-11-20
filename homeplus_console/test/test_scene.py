"""Planning-scene view: the OctoMap stream decoder (checked against hand-built streams), the worker and
the collision-overlay parser. No ROS needed."""
from __future__ import annotations

import base64
import math
import random
import struct

import numpy as np
import pytest

from homeplus_console.log_filter import LogBuffer
from homeplus_console.scene import (OctreeError, SceneWorker, compose, decode_octree, encode_octree,
                                    parse_collision, voxel_centres)

RES = 0.02


def full_node(logodds: float, children: int) -> bytes:
    """Full stream (OcTree::writeData): float32 log-odds, then one bit per allocated child."""
    return struct.pack("<fB", logodds, children)


def chain_full(first_child: int, then_child: int, leaf_depth: int, logodds: float) -> bytes:
    """Root → child `first_child` → child `then_child` at every depth below → a leaf at `leaf_depth`."""
    out = full_node(logodds, 1 << first_child)
    for _ in range(1, leaf_depth):
        out += full_node(logodds, 1 << then_child)
    return out + full_node(logodds, 0)


def centres(vox):
    return [tuple(round(v, 6) for v in row) for row in voxel_centres(vox, RES)]


# Key 32768 is the first key in the + half of every axis: from the root it is child 7 (+x +y +z), then
# child 0 at every lower depth (the remaining key bits are 0). Its leaf centre is +res/2 on each axis.
def test_full_single_leaf_at_origin():
    data = chain_full(7, 0, 16, 2.0)
    assert len(data) == 17 * 5
    vox, truncated = decode_octree(data, binary=False)
    assert not truncated
    assert vox.tolist() == [[0, 0, 0, 0]]
    assert centres(vox) == [(0.01, 0.01, 0.01, 0.02)]


def test_binary_single_leaf_at_origin():
    # OcTree::writeBinaryData: two bytes per inner node, 2 bits per child, children 0-3 in the first byte.
    data = bytes([0x00, 0xC0])          # root: child 7 = 11 (inner)
    data += bytes([0x03, 0x00]) * 14    # depths 1..14: child 0 = 11 (inner)
    data += bytes([0x02, 0x00])         # depth 15: child 0 = 10 (occupied leaf; bit 2i+1 set)
    vox, _ = decode_octree(data, binary=True)
    assert centres(vox) == [(0.01, 0.01, 0.01, 0.02)]


def test_binary_free_leaf_is_skipped_and_negative_x():
    # key x = 32767 (−x half, then +x at every depth: child 1), y = z = 32768 (child 6 at the root).
    data = bytes([0x00, 0x30])                     # root: child 6 inner
    data += bytes([0x0C, 0x00]) * 14               # child 1 inner
    data += bytes([0x01 | 0x08, 0x00])             # depth 15: child 0 free (01), child 1 occupied (10)
    vox, _ = decode_octree(data, binary=True)
    assert vox.tolist() == [[-1, 0, 0, 0]]
    assert centres(vox) == [(-0.01, 0.01, 0.01, 0.02)]


def test_full_pruned_leaf_has_level_and_size():
    # A leaf at depth 15 (pruned: 8 equal children merged) spans 2 keys per axis.
    vox, _ = decode_octree(chain_full(7, 0, 15, 1.0), binary=False)
    assert vox.tolist() == [[0, 0, 0, 1]]
    assert centres(vox) == [(0.02, 0.02, 0.02, 0.04)]


def test_full_threshold_and_two_children_in_order():
    # root mask 0xC0: child 6 then child 7. Child 6's subtree is free (log-odds −2) and is skipped.
    data = full_node(2.0, 0xC0)
    sub6 = b"".join(full_node(-2.0, 1 << 1) for _ in range(14)) + full_node(-2.0, 0)   # depth 1..15, leaf at 15
    sub7 = b"".join(full_node(2.0, 1) for _ in range(14)) + full_node(2.0, 0)
    vox, _ = decode_octree(data + sub6 + sub7, binary=False)
    assert vox.tolist() == [[0, 0, 0, 1]]
    vox, _ = decode_octree(data + sub6 + sub7, binary=False, occupancy_threshold=-5.0)
    assert sorted(vox.tolist()) == [[-2, 0, 0, 1], [0, 0, 0, 1]]


@pytest.mark.parametrize("binary", [False, True])
def test_roundtrip_random_cloud(binary):
    rng = random.Random(4)
    pts = {(rng.randint(-60, 60), rng.randint(-60, 60), rng.randint(0, 80)) for _ in range(300)}
    free = [((x + 0.5) * RES, (y + 0.5) * RES, (z + 0.5) * RES + 2.0) for x, y, z in list(pts)[:40]]
    data = encode_octree([((x + 0.5) * RES, (y + 0.5) * RES, (z + 0.5) * RES) for x, y, z in pts], RES, binary, free=free)
    vox, _ = decode_octree(data, binary=binary)
    assert {tuple(v[:3]) for v in vox.tolist()} == pts
    assert set(vox[:, 3].tolist()) == {0}


def test_encoder_matches_hand_stream():
    assert encode_octree([(0.01, 0.01, 0.01)], RES, binary=False) == chain_full(7, 0, 16, 2.0)
    assert encode_octree([(0.01, 0.01, 0.01)], RES, binary=True) == bytes([0x00, 0xC0]) + bytes([0x03, 0x00]) * 14 + bytes([0x02, 0x00])


def test_truncated_and_trailing_streams_raise():
    data = chain_full(7, 0, 16, 2.0)
    with pytest.raises(OctreeError):
        decode_octree(data[:-3], binary=False)
    with pytest.raises(OctreeError):
        decode_octree(data + b"\0", binary=False)
    assert decode_octree(b"", binary=True)[0].shape == (0, 4)


def test_voxel_cap_truncates():
    data = encode_octree([(i * RES, 0, 0) for i in range(50)], RES, binary=False)
    vox, truncated = decode_octree(data, binary=False, max_voxels=10)
    assert truncated and len(vox) == 10


def test_parse_collision_line():
    c = parse_collision("Collision overlay: palm_1 / detected_table at world [1.012, -0.034, 0.661]")
    assert c == {"a": "palm_1", "b": "detected_table", "frame": "world", "p": [1.012, -0.034, 0.661]}
    assert parse_collision("Plan for align is waiting for approval") is None


def test_compose_pose():
    yaw90 = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    out = compose({"p": [1.0, 0.0, 0.0], "q": yaw90}, {"p": [1.0, 0.0, 0.0], "q": [0, 0, 0, 1]})
    assert np.allclose(out["p"], [1.0, 1.0, 0.0])


def test_worker_polls_only_with_clients_and_decodes():
    calls = []
    oct_data = encode_octree([(0.5, 0.0, 0.7), (0.52, 0.0, 0.7)], RES, binary=False)

    def fetch():
        calls.append(1)
        return {"frame": "world",
                "octomap": {"id": "OcTree", "binary": False, "resolution": RES, "data": oct_data,
                            "origin": {"p": [0, 0, 0], "q": [0, 0, 0, 1]}},
                "objects": [{"id": "detected_table", "kind": "box", "dims": [0.6, 1.0, 0.04], "p": [1.2, 0, 0.64], "q": [0, 0, 0, 1]}],
                "attached": []}

    t = [100.0]
    logs = LogBuffer()
    w = SceneWorker(fetch, logs, clock=lambda: t[0])
    w.poll()
    v = w.view(t[0])
    assert v["octomap"]["count"] == 2 and v["octomap"]["format"] == "full" and not v["stale"]
    assert v["objects"][0]["id"] == "detected_table"
    vox = w.voxels()
    assert vox["count"] == 2 and vox["seq"] == v["octomap"]["seq"]
    rows = np.frombuffer(base64.b64decode(vox["data"]), "<i2").reshape(-1, 4)
    assert sorted(rows[:, 0].tolist()) == [25, 26]
    seq = vox["seq"]
    w.poll()  # unchanged octree: no re-decode, same seq
    assert w.voxels()["seq"] == seq
    t[0] += 30
    assert w.view(t[0])["stale"]
    logs.add("gesture_motion_executor", "WARN", "Collision overlay: palm_1 / detected_table at world [1.0, 0.0, 0.66]")
    w.scan_logs()
    assert w.view(t[0])["collision"]["b"] == "detected_table"


def test_worker_reports_errors():
    w = SceneWorker(lambda: {"error": "/get_planning_scene not available"}, LogBuffer())
    w.poll()
    v = w.view(0.0)
    assert v["error"] and v["stale"] and v["octomap"] is None
    w = SceneWorker(lambda: {"frame": "world", "octomap": {"id": "ColorOcTree", "binary": False, "resolution": RES,
                                                           "data": b"x"}, "objects": [], "attached": []}, LogBuffer())
    w.poll()
    assert "not supported" in w.view(0.0)["error"]
