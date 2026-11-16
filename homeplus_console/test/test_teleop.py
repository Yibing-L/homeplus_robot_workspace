"""Unit tests for the teleop pure logic: kinematics, controller limits, collision probe, governor.

    python3 -m pytest homeplus_console/test/test_teleop.py -q
"""
import math
import random

import pytest

from homeplus_console import robot_model
from homeplus_console import teleop_kin as kin
from homeplus_console.teleop import ORDER, Check, Controller, Intent, merged

D = math.radians
DT = 0.04


@pytest.fixture(scope="module")
def chain():
    return kin.ArmChain(robot_model.urdf_xml())


def home():
    q = {n: 0.0 for n in ORDER}
    q.update(joint_shoulder=D(40), joint_elbow=D(60), joint_hand=D(-10))
    return q


def run(ctrl, it, measured=None, check=None, n=25, t0=0.0, follow=True):
    """n ticks; the "robot" follows the command exactly when follow is set."""
    out = None
    m = dict(measured or ctrl.q)
    for i in range(n):
        out = ctrl.step(t0 + (i + 1) * DT, DT, it, m, check)
        if follow:
            m = dict(out.q)
    return out


def test_ik_round_trip(chain):
    random.seed(3)
    for _ in range(100):
        q = {"joint_shoulder": random.uniform(-0.3, 2.0), "joint_elbow": random.uniform(-0.4, 2.2),
             "joint_hand": random.uniform(-1.0, 1.2), "joint_wrist": 0.0}
        tgt = kin.task(chain.fk(q))
        q0 = {k: v + random.uniform(-0.05, 0.05) for k, v in q.items()}
        q0["joint_wrist"] = 0.0
        r = kin.solve(chain, q0, tgt)
        assert r.ok, (q, r)


def test_arm_is_planar_at_zero_roll(chain):
    a = chain.fk(home())
    b = chain.fk({**home(), "joint_shoulder": D(70), "joint_elbow": D(20), "joint_hand": D(30)})
    assert abs(a[1, 3] - b[1, 3]) < 1e-9   # in-plane joints never change the TCP's y


def test_base_forward_follows_heading(chain):
    c = Controller(chain)
    q = home()
    q["base_theta"] = D(90)
    c.anchor(q, 0.0)
    out = run(c, Intent(fwd=1.0), n=50)
    assert abs(out.q["base_x"]) < 1e-3 and out.q["base_y"] > 0.05   # facing +y, so forward is +y
    assert out.q["base_y"] <= 0.05 * 50 * DT + 1e-9                  # never faster than the cap


def test_strafe_and_turn_signs(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    out = run(c, Intent(strafe=1.0, turn=1.0), n=25)
    assert out.q["base_y"] > 0 and out.q["base_theta"] > 0


def test_mouse_reach_keeps_height_and_pitch(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    t0 = c.tcp()
    c.add_mouse(0, 0, 1.5, fast=False)       # wheel forward 1.5 notches → +1.5 cm reach
    out = run(c, Intent(drag=True), n=50)
    t1 = c.tcp(out.q)
    assert t1[0] - t0[0] == pytest.approx(0.015, abs=0.0015)
    assert abs(t1[1] - t0[1]) < 0.001 and abs(t1[2] - t0[2]) < D(0.5)


def test_mouse_up_raises_tcp(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    t0 = c.tcp()
    c.add_mouse(0, -100, 0, fast=False)      # screen up
    out = run(c, Intent(drag=True), n=50)
    assert c.tcp(out.q)[1] - t0[1] == pytest.approx(0.015, abs=0.0015)


def test_joint_speed_caps(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    cfg = merged(None)
    prev = dict(c.q)
    c.add_mouse(400, -400, 0, fast=True)
    for i in range(40):
        out = c.step((i + 1) * DT, DT, Intent(drag=True, fast=True, roll=1.0, grip=1.0), prev, None)
        for n, cap in cfg["joint_max_dps"].items():
            assert abs(out.q[n] - prev[n]) <= D(cap) * DT + 1e-9, n
        prev = out.q


def test_limits_are_respected(chain):
    c = Controller(chain)
    q = home()
    q["joint_grip_L"] = D(78)
    c.anchor(q, 0.0)
    out = run(c, Intent(grip=1.0), n=50)
    assert out.q["joint_grip_L"] <= D(80) + 1e-9
    out = run(c, Intent(roll=1.0), n=200)
    assert out.q["joint_wrist"] <= D(88) + 1e-9


def test_collision_probe_blocks_and_reports(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    wall = lambda q: Check(valid=q["base_x"] < 0.02, contacts=[] if q["base_x"] < 0.02 else [("base_link", "detected_table", 0.01)])  # noqa: E731
    out = run(c, Intent(fwd=1.0), check=wall, n=60)
    assert out.blocked and "detected_table" in out.blocked
    assert out.q["base_x"] < 0.02


def test_octomap_contacts_ignored_only_in_no_octomap_mode(chain):
    hit = lambda q: Check(valid=q["base_x"] < 0.01, contacts=[] if q["base_x"] < 0.01 else [("palm_1", "<octomap>", 0.01)])  # noqa: E731
    c = Controller(chain)
    c.anchor(home(), 0.0)
    assert run(c, Intent(fwd=1.0), check=hit, n=40).blocked
    c = Controller(chain, {"collision_mode": "no_octomap"})
    c.anchor(home(), 0.0)
    out = run(c, Intent(fwd=1.0), check=hit, n=40)
    assert not out.blocked and out.q["base_x"] > 0.01


def test_escape_from_contact_allowed_only_if_not_deeper(chain):
    # in contact with the table below; depth grows as the TCP goes down
    def table(q):
        z = kin.task(chain.fk(q))[1]
        depth = surface - z
        return Check(valid=depth <= 0, contacts=[("palm_1", "detected_table", depth)] if depth > 0 else [])
    c = Controller(chain)
    q = home()
    c.anchor(q, 0.0)
    z0 = c.tcp()[1]
    surface = z0 + 0.005   # starts 5 mm in contact
    c.add_mouse(0, -60, 0, fast=False)   # up = out of contact
    out = run(c, Intent(drag=True), check=table, n=30)
    assert not out.blocked and c.tcp(out.q)[1] > z0
    c.add_mouse(0, 200, 0, fast=False)   # down, into the table
    m, blocked = dict(c.q), []
    for i in range(40):
        out = c.step(3 + i * DT, DT, Intent(drag=True), m, table)
        m = out.q
        blocked.append(out.blocked)
    assert any(blocked)
    assert c.tcp(out.q)[1] >= surface - 1e-4   # stopped before re-entering contact


def test_governor_holds_when_robot_lags(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    stuck = dict(c.q)
    out = run(c, Intent(turn=1.0), measured=stuck, n=60, follow=False)
    assert "base_theta" in out.governed
    assert out.q["base_theta"] - stuck["base_theta"] < D(2.0) + D(20) * 0.35   # never runs away
    assert out.lost is None


def test_lost_when_far_from_command(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    m = dict(c.q)
    m["joint_elbow"] -= D(8)       # arm sagged: power lost
    out = c.step(DT, DT, Intent(), m, None)
    assert out.lost and "elbow" in out.lost


def test_release_stops_quickly(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    run(c, Intent(fwd=1.0), n=30)
    out = run(c, Intent(), n=10, t0=2.0)
    assert not out.moving and not c.busy()


def test_intent_parse_clamps():
    it = Intent.parse({"fwd": 5, "turn": -3, "keys": ["w"] * 40, "drag": 1})
    assert it.fwd == 1 and it.turn == -1 and len(it.keys) == 16 and it.drag


def test_probe_reaches_min_distance_so_base_stops_short(chain):
    c = Controller(chain)
    c.anchor(home(), 0.0)
    wall = lambda q: Check(valid=q["base_x"] < 0.10, contacts=[] if q["base_x"] < 0.10 else [("base_link", "detected_table", 0.01)])  # noqa: E731
    out = run(c, Intent(fwd=1.0), check=wall, n=200)
    assert out.blocked
    assert 0.10 - 0.035 < out.q["base_x"] < 0.10 - 0.025   # stops ~3 cm (probe_min_base_m) before contact


def test_gripper_may_close_on_an_object(chain):
    def cup(q):   # the fingers touch the cup once closed past 30 deg; pushing the arm forward goes deeper
        hit = q["joint_grip_L"] > D(30)
        depth = 0.002 + max(0.0, kin.task(chain.fk(q))[0] - reach0)
        return Check(valid=not hit, contacts=[("FingerL_new_1", "detected_cup", depth)] if hit else [])
    reach0 = kin.task(chain.fk(home()))[0]
    c = Controller(chain)
    c.anchor(home(), 0.0)
    out = run(c, Intent(grip=1.0), check=cup, n=40)
    assert not out.blocked and out.q["joint_grip_L"] > D(40)
    # but moving the arm while the fingers touch the cup is still blocked
    c.add_mouse(0, 0, 1.5, fast=False)
    blocked = [c.step(5.0 + i * DT, DT, Intent(drag=True), c.q, cup).blocked for i in range(10)]
    assert any(blocked)


def test_self_only_mode_ignores_world_but_not_self(chain):
    world = lambda q: Check(valid=False, contacts=[("palm_1", "detected_table", 0.01)])  # noqa: E731
    selfhit = lambda q: Check(valid=q["base_x"] <= 0, contacts=[("palm_1", "base_link", q["base_x"])] if q["base_x"] > 0 else [])  # noqa: E731
    c = Controller(chain, {"collision_mode": "self_only"})
    c.anchor(home(), 0.0)
    assert not run(c, Intent(fwd=1.0), check=world, n=5).blocked
    c = Controller(chain, {"collision_mode": "self_only"})
    c.anchor(home(), 0.0)
    assert run(c, Intent(fwd=1.0), check=selfhit, n=5).blocked
