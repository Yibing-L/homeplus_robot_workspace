"""hand_cup_to_person: scan headings, approach goal, pull detection, release rule,
config validation (handoff_logic.py), and the console's task-9 gating. No ROS."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'homeplus_moveit_config/scripts'))
sys.path.insert(0, str(ROOT / 'homeplus_console'))
from handoff_logic import (PullDetector, approach_goal, handoff_config_errors,  # noqa: E402
                           release_decision, scan_turns)
from homeplus_console.actions import HOLDING_TASKS, TASK_COMMANDS, ActionService  # noqa: E402
from homeplus_console.state_machine import SEARCH_TASKS  # noqa: E402


class ScanTurns(unittest.TestCase):
    def test_full_turn_in_30_degree_steps_looks_at_12_headings(self):
        turns = scan_turns(30, 360, 'left')
        self.assertEqual(len(turns), 11)  # 12th turn would return to the start heading
        self.assertAlmostEqual(sum(turns), math.radians(330))

    def test_partial_scan_and_direction(self):
        turns = scan_turns(30, 90, 'right')
        self.assertEqual(len(turns), 3)
        self.assertTrue(all(t < 0 for t in turns))

    def test_rejects_zero_step(self):
        with self.assertRaises(ValueError):
            scan_turns(0, 360)


class ApproachGoal(unittest.TestCase):
    def test_drives_along_the_line_and_faces_the_person(self):
        goal, _ = approach_goal((0.0, 0.0, 0.0), (2.0, 0.0), standoff_m=0.65, max_drive_m=2.5,
                                tcp_forward_m=0.5)
        self.assertAlmostEqual(goal['base_x'], 2.0 - 0.65 - 0.5)
        self.assertAlmostEqual(goal['base_y'], 0.0)
        self.assertAlmostEqual(goal['base_theta'], 0.0)

    def test_person_to_the_side_turns_first(self):
        goal, _ = approach_goal((1.0, 1.0, 0.0), (1.0, 3.0), 0.65, 2.5, tcp_forward_m=0.5)
        self.assertAlmostEqual(goal['base_theta'], math.pi / 2)
        self.assertAlmostEqual(goal['base_x'], 1.0)
        self.assertAlmostEqual(goal['base_y'], 3.0 - 1.15)

    def test_never_drives_toward_a_person_who_is_already_close(self):
        goal, why = approach_goal((0.0, 0.0, 0.0), (0.9, 0.0), 0.65, 2.5, tcp_forward_m=0.5)
        self.assertEqual((goal['base_x'], goal['base_y']), (0.0, 0.0))
        self.assertIn('drive 0.00', why)

    def test_heading_is_continuous_with_the_measured_one(self):
        # measured 3.0 rad, person behind-left at bearing -3.0 rad: turn ~+0.28, not -6.0
        goal, _ = approach_goal((0.0, 0.0, 3.0), (-2.0, -0.3), 0.65, 2.5)
        self.assertLess(abs(goal['base_theta'] - 3.0), 0.5)

    def test_refuses_a_long_drive(self):
        goal, why = approach_goal((0.0, 0.0, 0.0), (5.0, 0.0), 0.65, 2.5)
        self.assertIsNone(goal)
        self.assertIn('max_drive_m', why)


class Pull(unittest.TestCase):
    def feed(self, detector, values, t0=0.0, dt=0.02):
        hit = False
        for k, (shoulder, elbow) in enumerate(values):
            hit = detector.update(t0 + k * dt, {'joint_shoulder': shoulder, 'joint_elbow': elbow}) or hit
        return hit

    def make(self):
        return PullDetector(['joint_shoulder', 'joint_elbow'], math.radians(1.5), 0.3, baseline_sec=0.5)

    def test_hold_with_encoder_noise_is_not_a_pull(self):
        d = self.make()
        noise = [(math.radians(0.3 * ((k % 5) - 2) / 2), 0.0) for k in range(500)]
        self.assertFalse(self.feed(d, [(0.5 + a, 1.0 + b) for a, b in noise]))

    def test_sustained_deviation_is_a_pull(self):
        d = self.make()
        hold = [(0.5, 1.0)] * 40
        pulled = [(0.5 + math.radians(2.0), 1.0)] * 20  # 0.4 s at 2 deg
        self.assertTrue(self.feed(d, hold + pulled))

    def test_brief_bump_is_not_a_pull(self):
        d = self.make()
        hold = [(0.5, 1.0)] * 40
        bump = [(0.5, 1.0 - math.radians(3.0))] * 5  # 0.1 s
        self.assertFalse(self.feed(d, hold + bump + hold))

    def test_baseline_needs_time_first(self):
        d = self.make()
        # deviating from the very first sample: that IS the baseline, no pull
        self.assertFalse(self.feed(d, [(0.6, 1.0)] * 20))


class Release(unittest.TestCase):
    def test_rules(self):
        self.assertEqual(release_decision(False, False, None, True)[0], None)
        self.assertTrue(release_decision(True, False, None, True)[0])
        self.assertIsNone(release_decision(True, False, None, False)[0])
        self.assertTrue(release_decision(False, True, None, True)[0])
        self.assertTrue(release_decision(False, False, True, True)[0])
        # an operator Reject keeps the cup even with a pull
        self.assertFalse(release_decision(True, True, False, True)[0])

    def test_with_required_approval_only_the_operator_opens_the_gripper(self):
        # contact and pull alone never open it (operator, 2026-11-15)
        self.assertIsNone(release_decision(True, True, None, True, require_approval=True)[0])
        release, why = release_decision(True, True, True, True, require_approval=True)
        self.assertTrue(release)
        self.assertIn('pull', why)
        self.assertTrue(release_decision(False, False, True, True, require_approval=True)[0])
        self.assertFalse(release_decision(True, True, False, True, require_approval=True)[0])

    def test_map_wait_step_requires_approval(self):
        import yaml
        cmds = yaml.safe_load(open(ROOT / 'homeplus_moveit_config/config/gesture_execution_map.yaml'))['commands']
        steps = cmds['hand_cup_to_person']['steps']
        wait = next(s for s in steps if s['type'] == 'handoff_wait')
        self.assertTrue(wait['require_release_approval'])
        names = [s['name'] for s in steps]
        release = next(i for i, s in enumerate(steps) if s.get('detach_cup'))
        self.assertGreater(release, names.index(wait['name']))


class Config(unittest.TestCase):
    def test_wait_needs_an_earlier_guarded_step(self):
        errors = handoff_config_errors('s/wait', {'type': 'handoff_wait'}, 'handoff_wait', [])
        self.assertTrue(any('contact_from_step' in e for e in errors))
        self.assertEqual(handoff_config_errors('s/wait', {'type': 'handoff_wait', 'release_on_contact': False},
                                               'handoff_wait', []), [])
        errors = handoff_config_errors('s/wait', {'release_on_contact': False, 'operator_confirm': False},
                                       'handoff_wait', [])
        self.assertTrue(any('require_release_approval' in e for e in errors))

    def test_approach_limits(self):
        cfg = {'standoff_m': 0.2, 'velocity_scaling': 0.5}
        errors = handoff_config_errors('s/a', cfg, 'base_approach', [])
        self.assertEqual(len(errors), 2)

    def test_misplaced_keys(self):
        errors = handoff_config_errors('s/p', {'standoff_m': 0.6}, 'pose', [])
        self.assertTrue(errors)


class ConsoleGating(unittest.TestCase):
    def test_task_9_needs_the_cup(self):
        self.assertEqual(TASK_COMMANDS[9], 'hand_cup_to_person')
        self.assertIn(9, HOLDING_TASKS)
        self.assertIn(9, SEARCH_TASKS)
        self.assertFalse(ActionService._task_gate(9, False)['allowed'])
        self.assertTrue(ActionService._task_gate(9, True)['allowed'])
        self.assertFalse(ActionService._task_gate(1, True)['allowed'])


if __name__ == '__main__':
    unittest.main()
