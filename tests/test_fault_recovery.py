"""Offline tests for the executor's fault-recovery decisions (pure Python, no ROS)."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'homeplus_moveit_config/scripts'))
sys.path.insert(0, str(ROOT / 'homeplus_console'))
from fault_recovery import (MotionFault, fault_code, fault_retry_decision,  # noqa: E402
                            recovery_config_errors, recovery_scaling, recovery_setting,
                            scaled_step, start_state_recoverable)
from homeplus_console.log_filter import parse_status_text  # noqa: E402


class FaultCode(unittest.TestCase):
    def test_codes(self):
        self.assertEqual(fault_code('ERROR 123 TRACKING_ERROR dx_cm=0.1 shoulder_deg=3.04'), 'TRACKING_ERROR')
        self.assertEqual(fault_code('ERROR 9 SETTLE_TIMEOUT'), 'SETTLE_TIMEOUT')
        self.assertEqual(fault_code('ERROR 9 SERIAL [Errno 5] Input/output error'), 'SERIAL')
        self.assertEqual(fault_code('ERROR 9 HOST_STOP'), 'HOST_STOP')
        self.assertIsNone(fault_code('trajectory acknowledgement/endpoint verification failed'))
        self.assertIsNone(fault_code(None))
        self.assertIsNone(fault_code(''))


class Decision(unittest.TestCase):
    def test_retryable(self):
        for code in ('TRACKING_ERROR', 'SETTLE_TIMEOUT'):
            self.assertEqual(fault_retry_decision(code, 0, 2), (True, ''))
            self.assertTrue(fault_retry_decision(code, 1, 2)[0])
            self.assertFalse(fault_retry_decision(code, 2, 2)[0])
            self.assertFalse(fault_retry_decision(code, 0, 0)[0])

    def test_never_retried(self):
        for code in ('HOST_STOP', 'SERIAL', 'STALE_FEEDBACK', 'HOST_TIMEOUT', 'FIRMWARE_RESTART',
                     'NOT_READY_OR_BUSY', 'RX_ACK_TIMEOUT', None):
            self.assertFalse(fault_retry_decision(code, 0, 2)[0], code)

    def test_operator_stop_wins(self):
        retry, why = fault_retry_decision('TRACKING_ERROR', 0, 2, stop_requested=True)
        self.assertFalse(retry)
        self.assertIn('STOP', why)


class Scaling(unittest.TestCase):
    def test_halved_with_floor(self):
        self.assertAlmostEqual(recovery_scaling(1.0, 1), 0.5)
        self.assertAlmostEqual(recovery_scaling(1.0, 2), 0.25)
        self.assertAlmostEqual(recovery_scaling(0.15, 1), 0.1)   # floor
        self.assertAlmostEqual(recovery_scaling(0.05, 1), 0.05)  # never raised above the configured value

    def test_scaled_step(self):
        step = {'name': 'lift', 'velocity_scaling': 0.3, 'acceleration_scaling': 0.2, 'cartesian': True}
        out, percent = scaled_step(step, 1, 0.5)
        self.assertAlmostEqual(out['velocity_scaling'], 0.15)
        self.assertAlmostEqual(out['acceleration_scaling'], 0.1)
        self.assertEqual(percent, 50)
        self.assertAlmostEqual(out['_fault_scale'], 0.5)
        self.assertEqual(step['velocity_scaling'], 0.3)  # original untouched
        out, percent = scaled_step({'velocity_scaling': 0.15}, 1, 1.0)
        self.assertEqual(percent, 67)
        out, _ = scaled_step({}, 2, 0.5)  # Cartesian default 0.5 -> 0.125
        self.assertAlmostEqual(out['velocity_scaling'], 0.125)


class StartState(unittest.TestCase):
    def test_map_only(self):
        self.assertTrue(start_state_recoverable([('<octomap>', 'FingerL_new_1'), ('palm_1', '<octomap>')], True))

    def test_not_recoverable(self):
        self.assertFalse(start_state_recoverable([], True))  # constraint-only failure
        self.assertFalse(start_state_recoverable([('<octomap>', 'palm_1')], False))
        self.assertFalse(start_state_recoverable([('<octomap>', 'palm_1'), ('palm_1', 'detected_table')], True))
        self.assertFalse(start_state_recoverable([('link_elbow', 'link_base')], True))  # self-collision
        self.assertFalse(start_state_recoverable([('<octomap>', 'detected_table')], True))


class Settings(unittest.TestCase):
    def test_precedence(self):
        self.assertEqual(recovery_setting({}, {}, 'fault_retries'), 2)
        self.assertEqual(recovery_setting({}, {'fault_retries': 1}, 'fault_retries'), 1)
        self.assertEqual(recovery_setting({'fault_retries': 0}, {'fault_retries': 1}, 'fault_retries'), 0)
        self.assertEqual(recovery_setting({}, None, 'on_detection_timeout'), 'abort')
        self.assertTrue(recovery_setting({}, {}, 'start_state_recovery'))

    def test_validation(self):
        self.assertEqual(recovery_config_errors('s', {'fault_retries': 2, 'fault_settle_timeout_sec': 8,
                                                      'start_state_recovery': False,
                                                      'on_detection_timeout': 'back_up'}, True), [])
        self.assertTrue(recovery_config_errors('s', {'fault_retries': -1}, False))
        self.assertTrue(recovery_config_errors('s', {'fault_retries': True}, False))
        self.assertTrue(recovery_config_errors('s', {'fault_retries': 9}, False))
        self.assertTrue(recovery_config_errors('s', {'fault_settle_timeout_sec': 0}, False))
        self.assertTrue(recovery_config_errors('s', {'start_state_recovery': 'yes'}, False))
        self.assertTrue(recovery_config_errors('s', {'on_detection_timeout': 'back_up'}, False))
        self.assertTrue(recovery_config_errors('s', {'on_detection_timeout': 'spin'}, True))


class ConsoleParsing(unittest.TestCase):
    """New status lines must not be mistaken for step, abort or failure lines."""

    def test_new_lines_ignored_by_console_parser(self):
        for line in ('move_to_grasp: TRACKING_ERROR — recovering 1/2 at 50% speed',
                     'retract_arm_for_pickup: SETTLE_TIMEOUT — recovering 2/2 at 25% speed',
                     'lift: recovered after 1 retry',
                     'lift/arm: start touches map voxels only (<octomap>/palm_1) — clearing the map '
                     'and replanning once',
                     '[pick_up_coffee] detection: no stable detection on /molmopoint/target_pose '
                     'within 30s — backing up once and looking again'):
            self.assertIsNone(parse_status_text(line), line)

    def test_abort_reason_still_parsed(self):
        parsed = parse_status_text('Sequence pick_up_coffee ABORTED at lift: ERROR 5 TRACKING_ERROR '
                                   'shoulder_deg=3.1 (recovery: TRACKING_ERROR after 2 recovery attempt(s))')
        self.assertEqual(parsed['kind'], 'aborted')
        self.assertEqual(parsed['step'], 'lift')


class Fault(unittest.TestCase):
    def test_motion_fault_is_runtime_error(self):
        fault = MotionFault('x: base positioning failed', 'ERROR 4 SETTLE_TIMEOUT')
        self.assertIsInstance(fault, RuntimeError)
        self.assertEqual(fault.error, 'ERROR 4 SETTLE_TIMEOUT')
        self.assertIn('SETTLE_TIMEOUT', str(fault))


if __name__ == '__main__':
    unittest.main()
