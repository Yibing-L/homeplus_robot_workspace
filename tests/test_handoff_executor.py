"""hand_cup_to_person at the executor level with every ROS/MoveIt/Arduino call stubbed:
the scan turns until a detection, the approach goal, and the hand-off wait (approval
required, pull reported, timeout/STOP keep the cup).

Needs rclpy and the workspace messages (skipped otherwise); never starts a node.
"""
import math
import sys
import threading
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'homeplus_moveit_config/scripts'))
try:
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from gesture_motion_executor import GestureMotionExecutor, TRAJECTORY_JOINT_ORDER
except ImportError as exc:  # pragma: no cover
    raise unittest.SkipTest(f'ROS workspace not sourced: {exc}')


def joints(shoulder=0.5, elbow=1.0, theta=0.0):
    msg = JointState()
    msg.name = list(TRAJECTORY_JOINT_ORDER)
    q = dict.fromkeys(msg.name, 0.0)
    q.update(joint_shoulder=shoulder, joint_elbow=elbow, base_theta=theta)
    msg.position = [q[n] for n in msg.name]
    return msg


def executor():
    node = GestureMotionExecutor.__new__(GestureMotionExecutor)
    node.get_logger = lambda: Mock()
    node.base_frame = 'world'
    node.end_effector_link = 'grasp_tcp'
    node.dry_run = False
    node.latest_joint_state = joints()
    node._last_state_received = time.monotonic()
    node._joint_history = deque(maxlen=200)
    node._hardware_ready = True
    node._holding_cup = True
    node._stop_requested = threading.Event()
    node._plan_lock = threading.Lock()
    node._active_plan = None
    node._next_plan_request_id = 1
    node._trajectory_approval_event = threading.Event()
    node._trajectory_approval_lock = threading.Lock()
    node._trajectory_approval_response = None
    node.status = []
    node._publish_status = node.status.append
    node._publish_display_trajectory = Mock()
    node._measured_still = Mock(return_value=True)
    node._require_fresh_state = Mock()
    return node


def feed(node, stop, shoulder_at):
    """Joint-state publisher stand-in: 50 Hz until `stop` is set; shoulder_at(t) gives the angle."""
    t0 = time.monotonic()
    while not stop.is_set():
        now = time.monotonic()
        node.latest_joint_state = joints(shoulder=shoulder_at(now - t0))
        node._last_state_received = now
        node._joint_history.append((now, node.latest_joint_state))
        time.sleep(0.02)


WAIT = {'name': 'wait', 'type': 'handoff_wait', 'contact_from_step': 'lower', 'timeout_sec': 1.5,
        'require_release_approval': True}


class HandoffWait(unittest.TestCase):
    def run_wait(self, node, step, shoulder_at=lambda t: 0.5, contact=(), approve_after=None):
        stop = threading.Event()
        threading.Thread(target=feed, args=(node, stop, shoulder_at), daemon=True).start()
        if approve_after is not None:
            def approve():  # the console's Approve (Bool true on the approval topic)
                time.sleep(approve_after)
                GestureMotionExecutor._trajectory_approval_callback(node, SimpleNamespace(data=True))
            threading.Thread(target=approve, daemon=True).start()
        try:
            return node._handoff_wait('hand_cup_to_person', step['name'], step, set(contact))
        finally:
            stop.set()

    def test_contact_and_pull_alone_never_open_the_gripper(self):
        node = executor()
        pulled = lambda t: 0.5 + (math.radians(3.0) if t > 0.8 else 0.0)  # noqa: E731
        reason = self.run_wait(node, WAIT, shoulder_at=pulled, contact={'lower'})
        self.assertIn('not approved', reason)
        self.assertIn('still holding the cup', reason)
        self.assertTrue(any('pull on the cup measured' in s for s in node.status), node.status)
        self.assertTrue(any('Plan for wait/open_gripper is waiting for approval' in s for s in node.status))
        self.assertIsNone(node._active_plan)  # the approval prompt is withdrawn

    def test_operator_approval_opens_it(self):
        node = executor()
        reason = self.run_wait(node, WAIT, approve_after=0.4)
        self.assertIsNone(reason)
        self.assertTrue(any('release approved' in s for s in node.status), node.status)

    def test_reject_keeps_the_cup(self):
        node = executor()
        node._trajectory_approval_response = None
        stop = threading.Event()
        threading.Thread(target=feed, args=(node, stop, lambda t: 0.5), daemon=True).start()

        def reject():
            time.sleep(0.4)
            GestureMotionExecutor._trajectory_approval_callback(node, SimpleNamespace(data=False))
        threading.Thread(target=reject, daemon=True).start()
        try:
            reason = node._handoff_wait('hand_cup_to_person', 'wait', WAIT, set())
        finally:
            stop.set()
        self.assertIn('refused', reason)
        self.assertIn('still holding the cup', reason)

    def test_stop_keeps_the_cup(self):
        node = executor()
        threading.Timer(0.3, node._stop_requested.set).start()
        reason = self.run_wait(node, WAIT)
        self.assertTrue(reason.startswith('operator STOP'))

    def test_without_required_approval_a_pull_opens_it(self):
        node = executor()
        step = dict(WAIT, require_release_approval=False, release_on_contact=False, timeout_sec=3.0)
        pulled = lambda t: 0.5 + (math.radians(3.0) if t > 0.8 else 0.0)  # noqa: E731
        self.assertIsNone(self.run_wait(node, step, shoulder_at=pulled))


class ScanAndApproach(unittest.TestCase):
    def test_scan_turns_until_a_detection(self):
        node = executor()
        node.detection_min_stable_samples = 1
        found = PoseStamped()
        found.header.frame_id = 'world'
        found.pose.position.x = 2.0
        node._refresh_detection = Mock(side_effect=[None, None, found])
        goals = []

        def run_base_goal(seq, label, step, cfg, goal):
            goals.append(goal)
            node.latest_joint_state = joints(theta=goal['base_theta'])
            return None
        node._run_base_goal = run_base_goal
        step = {'name': 'scan', 'type': 'scan_rotate', 'step_deg': 30, 'max_deg': 360,
                'max_target_range_m': 4.0}
        pose, reason = node._scan_for_target('s', 'scan', step, {}, '/molmopoint/task_9/pose')
        self.assertIs(pose, found, reason)
        self.assertEqual(len(goals), 2)
        self.assertAlmostEqual(goals[-1]['base_theta'], math.radians(60))

    def test_scan_gives_up_after_the_last_heading(self):
        node = executor()
        node.detection_min_stable_samples = 1
        node._refresh_detection = Mock(return_value=None)
        node._run_base_goal = Mock(return_value=None)
        pose, reason = node._scan_for_target('s', 'scan', {'step_deg': 90, 'max_deg': 360}, {}, '/t')
        self.assertIsNone(pose)
        self.assertIn('4 heading(s)', reason)
        self.assertEqual(node._run_base_goal.call_count, 3)

    def test_approach_stops_with_the_cup_at_the_standoff(self):
        node = executor()
        tcp = SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=0.5, y=0.0, z=0.9)))
        node.tf_buffer = SimpleNamespace(lookup_transform=lambda *a, **k: tcp)
        node._run_base_goal = Mock(return_value=None)
        person = PoseStamped()
        person.pose.position.x, person.pose.position.y = 2.5, 0.0
        self.assertIsNone(node._approach_target('s', 'approach', {'standoff_m': 0.65}, {}, person))
        goal = node._run_base_goal.call_args[0][4]
        self.assertAlmostEqual(goal['base_x'], 2.5 - 0.65 - 0.5)


if __name__ == '__main__':
    unittest.main()
