"""Executor-level fault-recovery flow with every ROS/MoveIt/Arduino call stubbed.

Needs rclpy and the workspace messages (skipped otherwise); never starts a node.
"""
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'homeplus_moveit_config/scripts'))
try:
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from gesture_motion_executor import GestureMotionExecutor, TRAJECTORY_JOINT_ORDER
    from fault_recovery import MotionFault
except ImportError as exc:  # pragma: no cover
    raise unittest.SkipTest(f'ROS workspace not sourced: {exc}')

TRACKING = 'ERROR 41 TRACKING_ERROR dx_cm=0.000 dy_cm=0.000 yaw_deg=0.000 elbow_deg=0.2 shoulder_deg=3.04'
SETTLE = 'ERROR 42 SETTLE_TIMEOUT'


def state():
    msg = JointState()
    msg.name = list(TRAJECTORY_JOINT_ORDER)
    msg.position = [0.0] * len(msg.name)
    return msg


def executor():
    node = GestureMotionExecutor.__new__(GestureMotionExecutor)
    node.get_logger = lambda: Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=int(100e9), clock_type=ClockType.ROS_TIME))
    node.base_frame = 'world'
    node.end_effector_link = 'grasp_tcp'
    node.latest_joint_state = state()
    node._holding_cup = False
    node._cup_map_contact_allowed = False
    node._finger_map_contact_allowed = False
    node._stop_requested = threading.Event()
    node.enable_moveit_planning = True
    node.move_group_client = object()
    node.continue_on_arduino_timeout = False
    node.status = []
    node._publish_status = node.status.append
    node._start_sequence_trajectory_log = Mock()
    node._save_sequence_trajectory_csv = Mock()
    node._store_surface_height_memory = Mock()
    node._build_hardcoded_detection_pose = lambda cfg: None
    node._set_allowed_collisions = Mock()
    node._require_fresh_state = Mock()
    node._wait_for_fault_stillness = Mock(return_value=None)
    node._joint_goal_already_reached = Mock(return_value=False)
    node.joint_target_pub = Mock()
    node._wait_for_plan = Mock(return_value=True)
    node._plan_joint_state = Mock(return_value=object())
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    node._plan_sequence_pose = Mock(return_value=pose)
    node._fk_pose = Mock(return_value=pose)
    return node


def joint_seq(**step_extra):
    return {'type': 'sequence', 'steps': [
        {'name': 'retract', 'type': 'joint_state', 'group_name': 'arm', 'joints': {'joint_elbow': 0.5},
         **step_extra}]}


def pose_seq(**step_extra):
    return {'type': 'sequence', 'steps': [
        {'name': 'lift', 'type': 'pose', 'group_name': 'arm', 'cartesian': True,
         'position': [0.5, 0.0, 0.9], 'velocity_scaling': 0.3, 'acceleration_scaling': 0.3, **step_extra}]}


def aborted(node):
    return [s for s in node.status if 'ABORTED' in s]


class MotionFaultRecovery(unittest.TestCase):
    def test_joint_state_tracking_error_retried_at_half_speed(self):
        node = executor()
        node._await_step_trajectory = Mock(side_effect=[TRACKING, None])
        node._run_sequence('demo', joint_seq())
        self.assertEqual(node._plan_joint_state.call_count, 2)
        first, second = (c.args[2] for c in node._plan_joint_state.call_args_list)
        self.assertNotIn('velocity_scaling', first)
        self.assertAlmostEqual(second['velocity_scaling'], 0.5)
        # Same absolute goal both times (not rebuilt from the moved state).
        self.assertIs(node._plan_joint_state.call_args_list[0].args[1],
                      node._plan_joint_state.call_args_list[1].args[1])
        self.assertIn('retract: TRACKING_ERROR — recovering 1/2 at 50% speed', node.status)
        self.assertIn('retract: recovered after 1 retry', node.status)
        self.assertFalse(aborted(node))
        node._wait_for_fault_stillness.assert_called_once()

    def test_pose_gives_up_after_fault_retries(self):
        node = executor()
        node._await_step_trajectory = Mock(side_effect=[SETTLE, SETTLE, SETTLE])
        node._run_sequence('demo', pose_seq())
        scalings = [c.args[1].get('velocity_scaling') for c in node._plan_sequence_pose.call_args_list]
        self.assertEqual(len(scalings), 3)
        self.assertAlmostEqual(scalings[0], 0.3)
        self.assertAlmostEqual(scalings[1], 0.15)
        self.assertAlmostEqual(scalings[2], 0.1)  # 0.075 floored
        self.assertIn('lift: SETTLE_TIMEOUT — recovering 2/2 at 33% speed', node.status)
        self.assertEqual(len(aborted(node)), 1)
        self.assertIn('ERROR 42 SETTLE_TIMEOUT', aborted(node)[0])

    def test_sequence_level_fault_retries(self):
        node = executor()
        node._await_step_trajectory = Mock(side_effect=[TRACKING, TRACKING])
        node._run_sequence('demo', dict(pose_seq(), fault_retries=1))
        self.assertEqual(node._plan_sequence_pose.call_count, 2)
        self.assertTrue(aborted(node))

    def test_step_disables_retries(self):
        node = executor()
        node._await_step_trajectory = Mock(side_effect=[TRACKING])
        node._run_sequence('demo', dict(pose_seq(fault_retries=0), fault_retries=2))
        self.assertEqual(node._plan_sequence_pose.call_count, 1)
        self.assertTrue(aborted(node))
        node._wait_for_fault_stillness.assert_not_called()

    def test_stop_and_serial_never_retried(self):
        for error in ('ERROR 7 HOST_STOP', 'ERROR 7 SERIAL [Errno 5] Input/output error',
                      'ERROR 7 STALE_FEEDBACK', 'trajectory acknowledgement/endpoint verification failed'):
            node = executor()
            node._await_step_trajectory = Mock(side_effect=[error])
            node._run_sequence('demo', pose_seq())
            self.assertEqual(node._plan_sequence_pose.call_count, 1, error)
            self.assertEqual(aborted(node), [f'Sequence demo ABORTED at lift: {error}'])

    def test_operator_stop_blocks_retry(self):
        node = executor()
        node._stop_requested.set()
        node._run_sequence('demo', pose_seq())
        self.assertEqual(aborted(node), ['Sequence demo ABORTED at lift: operator STOP'])
        node._plan_sequence_pose.assert_not_called()

        node = executor()

        def fault_then_stop(*_):
            node._stop_requested.set()
            return TRACKING
        node._await_step_trajectory = Mock(side_effect=fault_then_stop)
        node._run_sequence('demo', pose_seq())
        self.assertEqual(node._plan_sequence_pose.call_count, 1)
        self.assertIn('operator STOP', aborted(node)[0])

    def test_contact_step_never_retried(self):
        node = executor()
        node._await_step_trajectory = Mock(side_effect=[TRACKING])
        node._run_sequence('demo', pose_seq(contact_ends_step=True))
        self.assertEqual(node._plan_sequence_pose.call_count, 1)
        self.assertFalse(aborted(node))
        self.assertIn('[demo] lift: contact detected', node.status)

    def test_bridge_not_ready_abandons_recovery(self):
        node = executor()
        node._wait_for_fault_stillness = Mock(return_value='bridge still not ready 10s after the fault')
        node._await_step_trajectory = Mock(side_effect=[TRACKING])
        node._run_sequence('demo', pose_seq())
        self.assertEqual(node._plan_sequence_pose.call_count, 1)
        self.assertIn('bridge still not ready', aborted(node)[0])

    def test_fault_inside_standoff_phase_retried(self):
        node = executor()
        pose = node._plan_sequence_pose.return_value
        node._plan_sequence_pose = Mock(side_effect=[MotionFault('lift: base positioning failed', SETTLE), pose])
        node._await_step_trajectory = Mock(return_value=None)
        node._run_sequence('demo', pose_seq())
        self.assertEqual(node._plan_sequence_pose.call_count, 2)
        self.assertAlmostEqual(node._plan_sequence_pose.call_args_list[1].args[1]['_fault_scale'], 0.5)
        self.assertFalse(aborted(node))

    def test_unretryable_fault_inside_standoff_raises_as_before(self):
        node = executor()
        node._plan_sequence_pose = Mock(side_effect=MotionFault('lift: base positioning failed', 'ERROR 3 HOST_STOP'))
        with self.assertRaises(RuntimeError):
            node._run_sequence('demo', pose_seq())

    def test_retry_skipped_when_fault_left_arm_at_goal(self):
        node = executor()
        node._joint_goal_already_reached = Mock(side_effect=[False, True])
        node._await_step_trajectory = Mock(side_effect=[TRACKING])
        node._run_sequence('demo', joint_seq())
        self.assertEqual(node._plan_joint_state.call_count, 1)
        self.assertFalse(aborted(node))


class Stillness(unittest.TestCase):
    def node(self, elbow_values, ready=True):
        node = GestureMotionExecutor.__new__(GestureMotionExecutor)
        node._hardware_ready = ready
        node._stop_requested = threading.Event()
        now = time.monotonic()
        history = []
        for k, elbow in enumerate(elbow_values):
            js = state()
            js.position[TRAJECTORY_JOINT_ORDER.index('joint_elbow')] = elbow
            history.append((now - 0.45 + 0.05 * k, js))
        node._joint_history = history
        node.latest_joint_state = history[-1][1]
        node._last_state_received = now
        return node

    def test_still_and_moving(self):
        self.assertTrue(self.node([0.1] * 10)._measured_still(0.0))
        self.assertFalse(self.node([0.1 + 0.01 * k for k in range(10)])._measured_still(0.0))
        # Samples from before the fault do not count.
        self.assertFalse(self.node([0.1] * 10)._measured_still(time.monotonic()))

    def test_wait_reports_bridge_not_ready(self):
        node = self.node([0.1] * 10, ready=False)
        self.assertIn('bridge still not ready', node._wait_for_fault_stillness('lift', 0.5))


class StartStateRecovery(unittest.TestCase):
    def node(self, results):
        node = executor()
        node._clear_and_rebuild_octomap = Mock(return_value=True)

        def valid(joints, group, constraints=None, log=False):
            ok, contacts = results.pop(0)
            node._last_invalid_state = None if ok else (contacts, True)
            return ok
        node._state_valid = Mock(side_effect=valid)
        node._last_state_received = time.monotonic()
        return node

    def test_map_only_contact_cleared_once(self):
        node = self.node([(False, [('<octomap>', 'palm_1')]), (True, [])])
        start = node._check_start_state('lift', state(), 'arm', None, {})
        self.assertIsNotNone(start)
        node._clear_and_rebuild_octomap.assert_called_once()
        self.assertTrue(node._clear_and_rebuild_octomap.call_args.kwargs['force'])
        self.assertTrue(any('clearing the map and replanning once' in s for s in node.status))

    def test_still_invalid_aborts_with_reason(self):
        node = self.node([(False, [('<octomap>', 'palm_1')]), (False, [('<octomap>', 'palm_1')])])
        self.assertIsNone(node._check_start_state('lift', state(), 'arm', None, {}))
        self.assertIn('still in collision after a map clear', node._plan_failure_reason)
        # Only once per step attempt.
        node._state_valid.side_effect = None
        node._state_valid.return_value = False
        self.assertIsNone(node._check_start_state('lift', state(), 'arm', None, {}))
        node._clear_and_rebuild_octomap.assert_called_once()

    def test_table_or_self_collision_not_cleared(self):
        for contacts in ([('<octomap>', 'palm_1'), ('palm_1', 'detected_table')],
                         [('link_elbow', 'link_shoulder')]):
            node = self.node([(False, contacts)])
            self.assertIsNone(node._check_start_state('lift', state(), 'arm', None, {}))
            node._clear_and_rebuild_octomap.assert_not_called()
            self.assertIn('start state invalid', node._plan_failure_reason)

    def test_disabled(self):
        node = self.node([(False, [('<octomap>', 'palm_1')])])
        self.assertIsNone(node._check_start_state('lift', state(), 'arm', None, {'start_state_recovery': False}))
        node._clear_and_rebuild_octomap.assert_not_called()


if __name__ == '__main__':
    unittest.main()
