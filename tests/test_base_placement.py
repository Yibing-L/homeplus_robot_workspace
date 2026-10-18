"""Offline tests for the arm reach model and deliberate base placement.

Pure Python + numpy against the URDF; no ROS needed. The executor-level tests
(config validation, reach offsets, detection gate) need rclpy and the
workspace's message packages and are skipped elsewhere. The MoveIt-side
checks (collisions, KDL, Cartesian approach) are in placement_moveit_check.py.
"""
import importlib
import math
import re
import struct
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'homeplus_moveit_config/scripts'))
from planar_reach import (PlanarArm, arm_plane_error, fixed_link_height,  # noqa: E402
                          placement_heading, plan_base_placements, wrap)

URDF = ROOT / 'homeplus_urdf_description/urdf/homeplus_urdf.xacro'
EXECUTION_MAP = ROOT / 'homeplus_moveit_config/config/gesture_execution_map.yaml'
MESHES = ROOT / 'homeplus_urdf_description/meshes'

# pick_up_coffee waypoints (forward, up) from the target: pre-grasp/align,
# grasp, lift, retreat. test_reach_offsets_follow_the_sequence checks that the
# executor derives exactly these from the YAML.
WAYPOINTS = [(-0.20, 0.0), (-0.20, 0.0), (-0.01, 0.0), (-0.01, 0.10), (-0.16, 0.10)]

# Situations from the 2026-10-18 step-5 failures: (base x, y, yaw, cup x, y, z).
# The cup is the far-view detection (the logged step-5 targets were 0.20 m
# short of it: 0.85-0.89 m x).
LOGGED = [
    (0.302, -0.040, 0.313, 1.137, 0.289, 0.807),
    (0.377, -0.040, 0.288, 1.109, 0.236, 0.745),
    (0.432, 0.001, 0.269, 1.096, 0.247, 0.773),
]


def urdf_text():
    # The xacro only adds includes (materials, transmissions, gazebo).
    return re.sub(r'<xacro:include[^>]*/>', '', URDF.read_text())


def load_placement_settings():
    steps = yaml.safe_load(EXECUTION_MAP.read_text())['commands']['pick_up_coffee']['steps']
    step = next(s for s in steps if s['name'] == 'move_to_pregrasp')
    return step['base_placement'], steps


def plan(arm, cup, base, heading_offsets=None):
    settings, _ = load_placement_settings()
    return plan_base_placements(
        arm, cup, base, WAYPOINTS,
        heading_offsets=[math.radians(v) for v in (heading_offsets or settings['heading_offsets_deg'])],
        min_margin=settings['min_reach_margin_m'],
        limit_margin=settings['joint_limit_margin_rad'],
        base_height=fixed_link_height(urdf_text()),
        preference=settings.get('reach_preference', 'centre'))


def tcp_world(arm, base_x, base_y, heading, q):
    """World pose of the TCP: base planar transform composed with arm FK."""
    t = arm.fk(q)
    c, s = math.cos(heading), math.sin(heading)
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return rz @ t[:3, 3] + np.array([base_x, base_y, 0.0]), rz @ t[:3, :3]


class ReachModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = PlanarArm.from_urdf(urdf_text())

    def test_structure_is_read_from_the_urdf(self):
        arm = self.arm
        self.assertEqual([j.name for j in arm.pitch_joints], ['joint_shoulder', 'joint_elbow', 'joint_hand'])
        # joint_wrist (axis -x) is a roll joint, held at zero for planar reach.
        self.assertEqual([j.name for j in arm.roll_joints], ['joint_wrist'])
        # The offset is the sum of the URDF y origins, not a constant here.
        expected = 0.05647 - 0.02219 + 0.01116 + 0.0372 - 0.026
        self.assertAlmostEqual(arm.lateral_offset, expected, places=9)
        self.assertAlmostEqual(fixed_link_height(urdf_text()), 0.0)

    def test_ignores_transmission_joint_references(self):
        # The expanded robot_description nests <joint name=...> inside
        # <transmission>; only robot-level joints describe the chain.
        text = urdf_text().replace('</robot>', '<transmission name="t"><type>x</type>'
                                   '<joint name="joint_hand"><hardwareInterface>p</hardwareInterface>'
                                   '</joint></transmission></robot>')
        self.assertAlmostEqual(PlanarArm.from_urdf(text).lateral_offset, self.arm.lateral_offset)
        self.assertAlmostEqual(fixed_link_height(text), 0.0)

    def test_plane_offset_is_independent_of_pitch_joints(self):
        rng = np.random.default_rng(1)
        for _ in range(50):
            q = {name: rng.uniform(lo, hi) for name, (lo, hi) in self.arm.limits.items()}
            q['joint_wrist'] = 0.0
            self.assertAlmostEqual(self.arm.fk(q)[1, 3], self.arm.lateral_offset, places=9)

    def test_closed_form_matches_fk_level_and_in_limits(self):
        solved = 0
        for forward in np.arange(0.2, 1.15, 0.025):
            for height in np.arange(0.5, 1.2, 0.025):
                for q in self.arm.solve_level(forward, height):
                    solved += 1
                    t = self.arm.fk(q)
                    np.testing.assert_allclose(t[:3, 3], [forward, self.arm.lateral_offset, height], atol=1e-9)
                    # Level: TCP x horizontal, z up (pitch and roll zero, no yaw).
                    np.testing.assert_allclose(t[:3, :3], np.eye(3), atol=1e-9)
                    for name, (lo, hi) in self.arm.limits.items():
                        self.assertTrue(lo <= q[name] <= hi, (name, q[name]))
                    self.assertEqual(q['joint_wrist'], 0.0)
        self.assertGreater(solved, 500)


class PlacementTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = PlanarArm.from_urdf(urdf_text())

    def check_placement(self, placement, cup):
        arm = self.arm
        # The arm plane passes through the cup and points at it.
        error, forward = arm_plane_error(placement.base_x, placement.base_y, placement.heading,
                                         cup[:2], arm.lateral_offset)
        self.assertAlmostEqual(error, 0.0, places=9)
        self.assertAlmostEqual(forward, placement.forward, places=9)
        # Every waypoint is reached level, in world, by the arm alone.
        for (dx, dz), q in zip(WAYPOINTS, placement.waypoints):
            position, rotation = tcp_world(arm, placement.base_x, placement.base_y, placement.heading, q)
            c, s = math.cos(placement.heading), math.sin(placement.heading)
            np.testing.assert_allclose(position, [cup[0] + c * dx, cup[1] + s * dx, cup[2] + dz], atol=1e-9)
            self.assertAlmostEqual(rotation[2, 0], 0.0, places=9)    # x axis horizontal
            self.assertAlmostEqual(rotation[2, 2], 1.0, places=9)    # z axis up
            self.assertAlmostEqual(math.atan2(rotation[1, 0], rotation[0, 0]), wrap(placement.heading), places=9)
            margin = load_placement_settings()[0]['joint_limit_margin_rad']
            for name, (lo, hi) in arm.limits.items():
                self.assertTrue(lo + margin - 1e-12 <= q[name] <= hi - margin + 1e-12, (name, q[name]))
        self.assertGreaterEqual(placement.margin, load_placement_settings()[0]['min_reach_margin_m'] - 1e-9)

    def test_logged_situations(self):
        for bx, by, byaw, cx, cy, cz in LOGGED:
            placements = plan(self.arm, (cx, cy, cz), (bx, by, byaw))
            self.assertTrue(placements, (bx, by, cx, cy, cz))
            for placement in placements:
                self.check_placement(placement, (cx, cy, cz))
            best = placements[0]
            self.assertEqual(best.heading_offset, 0.0)
            print(f'\n  logged cup ({cx:.3f}, {cy:.3f}, {cz:.3f}) from base ({bx:.3f}, {by:.3f}, {byaw:.3f}):'
                  f' base -> ({best.base_x:.3f}, {best.base_y:.3f}, {best.heading:.3f}),'
                  f' cup {best.forward:.3f} m ahead, margin {best.margin:.3f} m', end='')

    def test_near_preference(self):
        """'near' keeps exactly the configured margin on the near side and
        stands the base closer than 'centre' would."""
        settings, _ = load_placement_settings()
        self.assertEqual(settings['reach_preference'], 'near')
        for bx, by, byaw, cx, cy, cz in LOGGED:
            kwargs = dict(heading_offsets=[0.0], min_margin=settings['min_reach_margin_m'],
                          limit_margin=settings['joint_limit_margin_rad'])
            near = plan_base_placements(self.arm, (cx, cy, cz), (bx, by, byaw), WAYPOINTS, preference='near', **kwargs)
            centre = plan_base_placements(self.arm, (cx, cy, cz), (bx, by, byaw), WAYPOINTS, **kwargs)
            self.assertLess(near[0].forward, centre[0].forward)
            self.assertGreaterEqual(near[0].margin, settings['min_reach_margin_m'] - 1e-9)
            self.assertLess(near[0].margin, settings['min_reach_margin_m'] + 0.005 + 1e-9)
            # Alternatives move away from the near limit, never closer to it.
            self.assertEqual([p.forward for p in near], sorted(p.forward for p in near))
            print(f'\n  near: cup ({cx:.3f}, {cy:.3f}, {cz:.3f}) {near[0].forward:.3f} m ahead '
                  f'(pre-grasp TCP {near[0].forward + WAYPOINTS[0][0]:.3f}) vs centre {centre[0].forward:.3f}', end='')

    def test_direct_heading_drives_straight(self):
        """With the direct heading the base turns in place, then drives along
        its heading: no sideways component (turn_then_drive)."""
        for bx, by, byaw, cx, cy, cz in LOGGED:
            for placement in plan(self.arm, (cx, cy, cz), (bx, by, byaw), heading_offsets=[0.0]):
                dx, dy = placement.base_x - bx, placement.base_y - by
                sideways = -math.sin(placement.heading) * dx + math.cos(placement.heading) * dy
                self.assertAlmostEqual(sideways, 0.0, places=9)

    def test_sweep(self):
        """Every cup the arm can physically reach (some base pose exists) gets
        a valid placement. Reachability depends only on height: the base can
        stand anywhere, so x/y sweep the heading and base math."""
        reachable = []
        for cz in np.arange(0.55, 1.05, 0.01):
            any_pose = plan_base_placements(self.arm, (1.0, 0.2, cz), (0.0, 0.0, 0.0), WAYPOINTS,
                                            heading_offsets=[0.0], min_margin=0.0,
                                            limit_margin=load_placement_settings()[0]['joint_limit_margin_rad'])
            if any_pose and max(p.margin for p in any_pose) >= load_placement_settings()[0]['min_reach_margin_m']:
                reachable.append(round(cz, 3))
        self.assertTrue(reachable)
        print(f'\n  heights reachable with margin: {reachable[0]:.2f}-{reachable[-1]:.2f} m', end='')
        self.assertLessEqual(reachable[0], 0.70)
        self.assertGreaterEqual(reachable[-1], 0.90)
        count = 0
        for cz in np.linspace(reachable[0], reachable[-1], 7):
            for cx in (0.7, 1.1, 1.5):
                for cy in (-0.4, 0.2):
                    for byaw in (-0.5, 0.3):
                        placements = plan(self.arm, (cx, cy, cz), (0.3, 0.0, byaw))
                        self.assertTrue(placements, (cx, cy, cz, byaw))
                        self.check_placement(placements[0], (cx, cy, cz))
                        count += 1
        print(f'; {count} sweep cases placed', end='')

    def test_heading_wraps_near_pi(self):
        placements = plan(self.arm, (-1.0, 0.0, 0.78), (0.0, 0.0, 3.1))
        self.assertTrue(placements)
        self.check_placement(placements[0], (-1.0, 0.0, 0.78))

    def test_close_cup_backs_up(self):
        """A cup nearer than the reach interval puts the base further back."""
        placements = plan(self.arm, (0.6, 0.06, 0.78), (0.0, 0.0, 0.0), heading_offsets=[0.0])
        self.assertTrue(placements)
        self.assertLess(placements[0].base_x, 0.0)

    def test_heading_formula(self):
        d = self.arm.lateral_offset
        heading = placement_heading((0.0, 0.0), (1.0, 0.5), d)
        error, _ = arm_plane_error(0.0, 0.0, heading, (1.0, 0.5), d)
        self.assertAlmostEqual(error, 0.0, places=12)


def _stl_vertices(path):
    data = path.read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    facets = np.frombuffer(data[84:84 + count * 50],
                           dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return facets['v'].reshape(-1, 3).astype(float)


# The cup the pickup was tuned for (paper coffee cup, ~8 cm across). The
# detector only bounds radius to 0.025-0.065 m; measure the real cup.
NOMINAL_CUP_RADIUS_M = 0.04


class LateralThresholdTest(unittest.TestCase):
    """Justify max_arm_plane_error_m from the finger meshes.

    In the TCP frame both finger axes are vertical, so closure is planar at
    grasp height. A cup of radius r sits with its front surface where
    move_to_grasp puts it (offset ahead of the TCP), centred e off the arm
    plane. Holding it fixed (conservative: really the first finger pushes it
    towards the centre), both fingers must touch it before the configured
    close angle, on opposite sides of its centre.
    """

    @classmethod
    def setUpClass(cls):
        import xml.etree.ElementTree as ET
        root = ET.fromstring(urdf_text())
        joints = {j.get('name'): j for j in root.iter('joint')}
        links = {l.get('name'): l for l in root.iter('link')}
        steps = yaml.safe_load(EXECUTION_MAP.read_text())['commands']['pick_up_coffee']['steps']
        cls.closed = math.radians(next(s for s in steps if s['name'] == 'close_gripper_at_target')['value'])
        cls.limit = next(s for s in steps if s['name'] == 'refine_target')['max_arm_plane_error_m']
        # Front surface ahead of the TCP at the grasp (the detected point is
        # the visible surface; cup_surface_to_center_m is 0).
        cls.surface_ahead = -next(s for s in steps if s['name'] == 'move_to_grasp')['offset_x']

        def vec(text):
            return np.array([float(v) for v in text.split()])
        tcp = joints['palm_to_grasp_tcp'].find('origin')
        tcp_xyz, tcp_pitch = vec(tcp.get('xyz')), vec(tcp.get('rpy'))[1]
        c, s = math.cos(tcp_pitch), math.sin(tcp_pitch)
        tcp_rot = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        cls.angles = np.radians(np.arange(0.0, math.degrees(cls.closed) + 1e-9, 0.5))
        cls.slices = {}
        for side, joint_name, link_name in (('L', 'joint_grip_L', 'FingerL_new_1'),
                                            ('R', 'joint_grip_R', 'FingerR_new_1')):
            joint = joints[joint_name]
            axis = vec(joint.find('axis').get('xyz'))
            axis /= np.linalg.norm(axis)
            np.testing.assert_allclose(np.abs(tcp_rot.T @ axis), [0, 0, 1], atol=1e-3)
            k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            mesh = _stl_vertices(MESHES / f'{link_name}.stl') * 0.001 + vec(links[link_name].find('collision/origin').get('xyz'))
            cls.slices[side] = []
            for angle in cls.angles:
                rot = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)
                v = (rot @ mesh.T).T + vec(joint.find('origin').get('xyz'))
                v = (tcp_rot.T @ (v - tcp_xyz).T).T
                cls.slices[side].append(v[np.abs(v[:, 2]) < 0.015][:, :2])

    def first_contact(self, side, centre, radius):
        for points in self.slices[side]:
            d = np.hypot(points[:, 0] - centre[0], points[:, 1] - centre[1])
            if d.min() < radius:
                return points[d.argmin()]
        return None

    def max_offset(self, radius):
        centre_x = self.surface_ahead + radius
        best = 0.0
        for e in np.arange(0.0, 0.08, 0.0025):
            left = self.first_contact('L', (centre_x, e), radius)
            right = self.first_contact('R', (centre_x, e), radius)
            if left is None or right is None or not right[1] < e < left[1]:
                break
            best = e
        return best

    def test_threshold_keeps_both_fingers_on_the_cup(self):
        offsets = {r: self.max_offset(r) for r in (0.03, NOMINAL_CUP_RADIUS_M, 0.05, 0.065)}
        print('\n  max lateral offset with both fingers on opposite sides before close: '
              + ', '.join(f'r={r:.3f}: {e * 1000:.1f} mm' for r, e in offsets.items())
              + f'; threshold {self.limit * 1000:.0f} mm', end='')
        # Leave 5 mm for the stability gate's residual detection noise.
        self.assertLessEqual(self.limit + 0.005, offsets[NOMINAL_CUP_RADIUS_M])


def _executor_module():
    try:
        return importlib.import_module('gesture_motion_executor')
    except ImportError as exc:  # rclpy / moveit_msgs missing off the robot machine
        raise unittest.SkipTest(f'executor import needs the ROS workspace: {exc}')


class ExecutorConfigTest(unittest.TestCase):
    def test_execution_map_validates(self):
        module = _executor_module()
        commands = yaml.safe_load(EXECUTION_MAP.read_text())['commands']
        for name, cfg in commands.items():
            self.assertEqual(module.GestureMotionExecutor._command_config_errors(name, cfg), [], name)
        self.assertIn('press_square_button', commands)

    def test_rejects_bad_placement_config(self):
        module = _executor_module()
        errors = module.GestureMotionExecutor._command_config_errors('bad', {
            'type': 'pose', 'target': 'detected', 'group_name': 'base_arm', 'mobile_standoff': True,
            'base_placement': {'base_motion': 'teleport', 'typo': 1}, 'base_goal_tcp_x_min_m': 0.5,
            'base_goal_tcp_x_max_m': 0.7})
        self.assertTrue(any('base_motion' in e for e in errors))
        self.assertTrue(any('typo' in e for e in errors))
        self.assertTrue(any('base_goal_tcp' in e for e in errors))
        errors = module.GestureMotionExecutor._command_config_errors('seq', {
            'type': 'sequence', 'detection_topic': '/x', 'steps': [
                {'name': 'look', 'type': 'detect', 'max_arm_plane_error_m': 0.02, 'redo_from_step': 'later'},
                {'name': 'later', 'type': 'gripper', 'value': 0.0}]})
        self.assertTrue(any('earlier step' in e for e in errors))

    def test_refine_reach_check(self):
        """refine_target re-places the base exactly when the remaining
        arm-only waypoints leave the arm's joint limits (no extra margin: the
        placement's joint_limit_margin_rad is not applied here)."""
        module = _executor_module()
        from geometry_msgs.msg import PoseStamped
        _, steps = load_placement_settings()
        arm = PlanarArm.from_urdf(urdf_text())
        fake = SimpleNamespace(end_effector_link='grasp_tcp', _base_link_height=0.0,
                               _arm_model=lambda link=None: arm)
        fake._arm_reach_offsets = lambda s, k: module.GestureMotionExecutor._arm_reach_offsets(fake, s, k)
        refine = next(i for i, s in enumerate(steps) if s['name'] == 'refine_target')
        bx, by, byaw, cx, cy, cz = LOGGED[1]
        placement = plan(arm, (cx, cy, cz), (bx, by, byaw))[0]
        target = PoseStamped()
        target.pose.position.z = cz
        check = module.GestureMotionExecutor._arm_only_reachable
        self.assertTrue(check(fake, steps, refine, target, placement.forward))
        # Near-side limit of arm-only reach at zero joint margin.
        remaining = WAYPOINTS[1:]
        limit = next(f for f in np.arange(placement.forward, 0.0, -0.001)
                     if not any(arm.solve_path([(f + dx, cz + dz) for dx, dz in remaining], b, 0.0)
                                for b in (1, -1)))
        print(f'\n  near placement {placement.forward:.3f} m; arm-only reach ends at {limit + 0.001:.3f} m '
              f'(a close-up cup up to {(placement.forward - limit - 0.001) * 100:.1f} cm nearer needs no re-placement)', end='')
        self.assertTrue(check(fake, steps, refine, target, limit + 0.002))
        self.assertFalse(check(fake, steps, refine, target, limit - 0.002))

    def test_reach_offsets_follow_the_sequence(self):
        module = _executor_module()
        _, steps = load_placement_settings()
        fake = SimpleNamespace(end_effector_link='grasp_tcp')
        index = next(i for i, s in enumerate(steps) if s['name'] == 'move_to_pregrasp')
        offsets = module.GestureMotionExecutor._arm_reach_offsets(fake, steps, index)
        np.testing.assert_allclose(offsets, WAYPOINTS, atol=1e-12)


class DetectionGateTest(unittest.TestCase):
    """_wait_for_detection must reject outliers, not average them in."""

    def make(self, module, points, stamp_step=1.0):
        from geometry_msgs.msg import PoseStamped
        now = 1000.0
        samples = []
        for k, (x, y, z) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            t = now - (len(points) - k) * stamp_step * 0.1
            pose.header.stamp.sec = int(t)
            pose.header.stamp.nanosec = int((t % 1) * 1e9)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.0
            samples.append(pose)
        node = module.GestureMotionExecutor.__new__(module.GestureMotionExecutor)
        node.detection_min_stable_samples = 1
        node.detection_stability_tolerance_m = 0.05
        node.detection_max_refresh_shift_m = 0.0
        node._detection_lock = threading.Lock()
        node._detection_samples = {'/t': samples}
        logs = []
        node.get_logger = lambda: SimpleNamespace(info=logs.append, warn=logs.append, warning=logs.append)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now * 1e9)))
        node.get_parameter = lambda name: SimpleNamespace(value=15.0)
        return node, logs

    def test_outlier_is_not_averaged(self):
        module = _executor_module()
        good = [(1.0, 0.2, 0.78), (1.004, 0.198, 0.781), (0.998, 0.203, 0.779)]
        node, _ = self.make(module, good[:2] + [(1.08, 0.25, 0.70)] + good)
        pose = node._wait_for_detection('/t', 0.5, required_samples=3, stability_tolerance_m=0.02)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.pose.position.x, 1.0, places=6)
        self.assertAlmostEqual(pose.pose.position.y, 0.2, places=6)

    def test_outlier_in_window_blocks(self):
        module = _executor_module()
        node, _ = self.make(module, [(1.0, 0.2, 0.78), (1.004, 0.198, 0.781), (1.08, 0.25, 0.70)])
        self.assertIsNone(node._wait_for_detection('/t', 0.3, required_samples=3, stability_tolerance_m=0.02))


if __name__ == '__main__':
    unittest.main(verbosity=2)
