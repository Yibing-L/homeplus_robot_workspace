"""Offline regression tests; never start ROS nodes or open a serial device."""
import importlib.util
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from collections import deque

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'homeplus_moveit_config/scripts'), str(ROOT/'homeplus_vision')]
from arduino_reader import ArduinoBridge, JOINT_NAMES, from_hardware, validate_command
from gesture_motion_executor import GestureMotionExecutor, TRAJECTORY_JOINT_ORDER, PlanContext
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.time import Time
from rclpy.clock import ClockType
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from homeplus_vision.target_geometry import fit_cup_center, fit_button_plane, offset_surface_goal
from homeplus_vision.obstacle_cloud_node import ObstacleCloudNode


def executor():
    node = GestureMotionExecutor.__new__(GestureMotionExecutor)
    node.get_logger = lambda: Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=int(100e9), clock_type=ClockType.ROS_TIME))
    node.get_parameter = lambda name: SimpleNamespace(value={'detection_max_age_sec': 15, 'joint_state_max_age_sec': 0.5}[name])
    node.base_frame = 'world'
    node.default_group_name = 'base_arm'
    node.latest_joint_state = state()
    node._last_state_received = time.monotonic()
    node._joint_history = []
    return node


def state(values=None):
    msg = JointState()
    msg.name = list(JOINT_NAMES)
    msg.position = [0.0]*8 if values is None else list(values)
    msg.header.stamp = Time(nanoseconds=int(100e9)).to_msg()
    return msg


def command(points):
    return 'CMD 123 ' + ','.join(' '.join(map(str,p)) for p in points)


class ProtocolTests(unittest.TestCase):
    def test_si_roundtrip(self):
        q = from_hardware([100, -20, 90, 70, -20, 30, 45, 60])
        self.assertAlmostEqual(q[0], 1)
        self.assertAlmostEqual(q[2], math.pi/2)
        self.assertAlmostEqual(q[3], math.radians(70))

    def test_protocol_rejects_unsafe_queues(self):
        good = [0]*11
        self.assertEqual(validate_command(command([good])), (123, 0))
        cases = [[good, good], [[0]*10], [[math.nan]+[0]*10], [[0]*6+[145]+[0]*4], [good]*46]
        for points in cases:
            with self.subTest(points=points[:2]), self.assertRaises(ValueError):
                validate_command(command(points))
        with self.assertRaises(ValueError):
            validate_command('CMD 123 ' + '0 '*2048)

    def test_stale_done_does_not_complete_active_command(self):
        bridge = ArduinoBridge.__new__(ArduinoBridge)
        bridge.active_id = 42
        bridge.ack_pub, bridge.status_pub = Mock(), Mock()
        bridge.get_logger = lambda: Mock()
        bridge._handle_line('DONE 41')
        bridge._handle_line('DONE')
        bridge._handle_line('ERROR 41 SETTLE_TIMEOUT')
        bridge.ack_pub.publish.assert_not_called()
        self.assertEqual(bridge.active_id, 42)

    def test_done_does_not_publish_commanded_positions(self):
        bridge = ArduinoBridge.__new__(ArduinoBridge)
        bridge.active_id = 42
        bridge.last_feedback = time.monotonic()
        bridge.ack_pub, bridge.status_pub, bridge.joint_state_pub = Mock(), Mock(), Mock()
        bridge._set_ready = Mock()
        bridge._handle_line('DONE 42')
        bridge.joint_state_pub.publish.assert_not_called()
        self.assertIsNone(bridge.active_id)

    def test_partial_serial_lines_are_not_state(self):
        bridge = ArduinoBridge.__new__(ArduinoBridge)
        bridge._handle_line('USB Serial Ready')
        with self.assertRaises(ValueError):
            bridge._handle_line('STATE2 42 0 0')

    def test_no_startup_transmission_until_firmware_feedback(self):
        bridge = ArduinoBridge.__new__(ArduinoBridge)
        bridge.ser = Mock()
        bridge.ser.in_waiting = 0
        bridge.ser.read.return_value = b''
        bridge.compatible = False
        bridge.active_id = None
        bridge.last_ping = 0.0
        bridge.rx = bytearray()
        bridge.serial_bytes_received = 0
        with patch('arduino_reader.time.monotonic', return_value=100.0):
            bridge._read_loop()
            bridge.ser.write.assert_not_called()
            bridge.compatible = True
            bridge.last_feedback = 100.0
            bridge._read_loop()
        bridge.ser.write.assert_called_once_with(b'PING\n')

    def test_feedback_startup_requires_stillness_and_publishes_actual_values(self):
        bridge=ArduinoBridge.__new__(ArduinoBridge)
        bridge.active_id=None;bridge.compatible=False;bridge.ready=False
        bridge.state_history=deque(maxlen=30)
        bridge.joint_state_pub=Mock();bridge.hardware_state_pub=Mock()
        bridge.ready_pub=Mock();bridge.status_pub=Mock();bridge.get_logger=lambda:Mock()
        bridge.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=int(100e9)))
        with patch('arduino_reader.time.monotonic',side_effect=[1+i*.1 for i in range(9)]):
            bridge._handle_line('STATE2 0 100 0 0 0 0 0 10 20')
            self.assertFalse(bridge.ready)
            for _ in range(8):bridge._handle_line('STATE2 0 100 0 0 0 0 0 10 20')
        self.assertTrue(bridge.ready)
        js=bridge.hardware_state_pub.publish.call_args.args[0]
        self.assertAlmostEqual(js.position[0],1.)
        self.assertAlmostEqual(js.position[7],math.radians(20))


class ExecutorTests(unittest.TestCase):
    def test_feedback_is_accepted_while_busy(self):
        node = executor(); node._busy = True
        msg = state([0.1]*8)
        node.joint_states_callback(msg)
        self.assertEqual(node.latest_joint_state.position[0], 0.1)

    def test_failure_retains_measured_state(self):
        node = executor(); before = node.latest_joint_state
        node._pending_joint_state = [1.0]*8
        node._pending_joint_state_lock = threading.Lock()
        node.stop_pub = Mock()
        node._force_apply_pending_joint_state('test timeout')
        self.assertIs(node.latest_joint_state, before)
        self.assertIsNone(node._pending_joint_state)
        node.stop_pub.publish.assert_called_once()

    def test_detected_height_is_not_clamped_to_a_fixed_table(self):
        node = executor(); p = PoseStamped(); p.header.frame_id='world'
        commands = node._load_execution_map(str(ROOT/'homeplus_moveit_config/config/gesture_execution_map.yaml'))
        pickup = commands['pick_up_coffee']
        self.assertNotIn('support_surface_z_m', pickup)
        step = next(s for s in pickup['steps'] if s['name']=='move_to_pregrasp')
        for z in (.5, .73, .95):
            p.pose.position.z=z
            result=node._resolve_detected_pose(step,p)
            self.assertAlmostEqual(result['position'][2],z)
        errors=node._command_config_errors('old',dict(type='pose',target='detected',support_surface_z_m=.72))
        self.assertTrue(errors) # Stale configuration must not silently restore a clamp.

    def test_offset_rotates_with_target(self):
        node = executor(); p=PoseStamped();p.header.frame_id='world'
        p.pose.orientation.z=math.sqrt(0.5);p.pose.orientation.w=math.sqrt(0.5)
        out=node._resolve_detected_pose({'offset_x':-0.1,'offset_frame':'target','orientation_from_detection':True},p)
        self.assertAlmostEqual(out['position'][0],0)
        self.assertAlmostEqual(out['position'][1],-0.1)
        self.assertNotIn('rpy_deg',out)

    def test_detection_rejects_old_duplicate_and_wrong_frame(self):
        node=executor();node._detection_after_ns=int(90e9);node._detection_lock=threading.Lock()
        node._detection_samples={};node._detection_poses={}
        p=PoseStamped();p.header.frame_id='world';p.header.stamp=Time(nanoseconds=int(95e9)).to_msg()
        node._detection_pose_callback('cup',p);node._detection_pose_callback('cup',p)
        self.assertEqual(len(node._detection_samples['cup']),1)
        p.header.stamp=Time(nanoseconds=int(89e9)).to_msg();node._detection_pose_callback('cup',p)
        p.header.stamp=Time(nanoseconds=int(96e9)).to_msg();p.header.frame_id='camera';node._detection_pose_callback('cup',p)
        self.assertEqual(len(node._detection_samples['cup']),1)

    def test_goal_requires_measured_dwell_not_done_alone(self):
        node=executor();now=time.monotonic();node._command_started=now-2;node._pending_joint_state=[0.0]*8
        node._joint_history=[(now-d,state()) for d in (0.6,0.4,0.2,0)]
        self.assertTrue(node._measured_goal_reached())
        node._joint_history[0][1].position[7]=math.radians(2.5)
        self.assertFalse(node._measured_goal_reached())

    def test_collision_overlay_is_read_only_and_replaces_old_markers(self):
        from moveit_msgs.msg import RobotState, ContactInformation
        from visualization_msgs.msg import Marker
        node=executor();node.collision_state_pub=Mock();node.collision_markers_pub=Mock()
        node.arduino_cmd_pub=Mock();node.joint_state_pub=Mock()
        original=list(node.latest_joint_state.position)
        rejected=RobotState(joint_state=state([.2]*8),is_diff=True)
        contact=ContactInformation(contact_body_1='<octomap>',body_type_1=ContactInformation.WORLD_OBJECT,
                                   contact_body_2='palm_1',body_type_2=ContactInformation.ROBOT_LINK,depth=.002)
        contact.header.frame_id='world';contact.position.x=1.;contact.position.z=.73
        node._publish_collision_debug(rejected,[contact])
        display=node.collision_state_pub.publish.call_args.args[0]
        self.assertFalse(display.hide)
        self.assertEqual(list(display.state.joint_state.position),list(rejected.joint_state.position))
        self.assertEqual([c.id for c in display.highlight_links],['palm_1'])
        markers=node.collision_markers_pub.publish.call_args.args[0].markers
        self.assertEqual(markers[0].action,Marker.DELETEALL)
        self.assertEqual(markers[1].pose.position.z,.73)
        self.assertIn('<octomap> / palm_1',markers[2].text)
        self.assertEqual(list(node.latest_joint_state.position),original)
        node.arduino_cmd_pub.publish.assert_not_called();node.joint_state_pub.publish.assert_not_called()
        node._republish_collision_debug()
        self.assertEqual(node.collision_state_pub.publish.call_count,2)
        node._clear_collision_debug()
        self.assertTrue(node.collision_state_pub.publish.call_args.args[0].hide)
        self.assertEqual(list(node.collision_state_pub.publish.call_args.args[0].state.joint_state.name),
                         list(node.latest_joint_state.name))
        self.assertEqual(len(node.collision_markers_pub.publish.call_args.args[0].markers),1)

    def test_clearing_overlay_before_feedback_never_publishes_empty_state(self):
        node = executor()
        node.latest_joint_state = None
        node.collision_state_pub = Mock()
        node.collision_markers_pub = Mock()
        node._collision_debug = None
        node._clear_collision_debug()
        node._republish_collision_debug()
        node.collision_state_pub.publish.assert_not_called()
        self.assertEqual(node.collision_markers_pub.publish.call_count, 2)

    def test_execution_failure_logs_residual_and_jitter_without_altering_state(self):
        node = executor()
        logger = Mock()
        node.get_logger = lambda: logger
        node.latest_joint_state.position[7] = math.radians(65.0)
        node._pending_joint_state = [0.0]*7 + [math.radians(68.748)]
        node._endpoint_profile = 'transit'
        now = time.monotonic()
        node._last_state_received = now
        node._joint_history = [(now, state(node.latest_joint_state.position))]
        original = list(node.latest_joint_state.position)
        node._log_execution_feedback('SETTLE_TIMEOUT')
        messages = '\n'.join(call.args[0] for call in logger.error.call_args_list)
        self.assertIn('profile=transit', messages)
        self.assertIn('joint_shoulder: target=68.748 feedback=65.000 error=+3.748 deg', messages)
        self.assertIn('feedback span over last 0.7s=0.000 deg', messages)
        self.assertEqual(list(node.latest_joint_state.position), original)
        self.assertAlmostEqual(node._pending_joint_state[7], math.radians(68.748))

    def test_rviz_has_collision_overlay_and_camera_context(self):
        config=yaml.safe_load((ROOT/'homeplus_moveit_config/config/moveit.rviz').read_text())
        displays=config['Visualization Manager']['Displays']
        state_display=next(d for d in displays if d['Class']=='moveit_rviz_plugin/RobotState')
        self.assertEqual(state_display['Robot State Topic'],'/planning_debug/rejected_state')
        topics={d.get('Topic',{}).get('Value') for d in displays if isinstance(d.get('Topic',{}),dict)}
        self.assertTrue({'/obstacle_cloud','/object_mask','/vision/clouds/d405/target'}.issubset(topics))

    def test_failed_ik_candidates_run_diagnostic_without_returning_a_goal(self):
        from moveit_msgs.msg import Constraints
        node=executor();node.end_effector_link='grasp_tcp'
        node.ik_client=Mock();node._call_service=Mock(return_value=SimpleNamespace(error_code=SimpleNamespace(val=-31)))
        node._diagnose_continuation_ik=Mock()
        result=node._ik_candidates(PoseStamped(),{'group_name':'arm','ik_candidate_attempts':1},Constraints())
        self.assertEqual(result,[])
        node._diagnose_continuation_ik.assert_called_once()
        request=node._diagnose_continuation_ik.call_args.args[0]
        self.assertTrue(request.ik_request.avoid_collisions)

    def test_ik_failure_does_not_call_rrt(self):
        node=executor();node.move_group_client=Mock();node.move_group_client.server_is_ready.return_value=True
        node._clear_and_rebuild_map=lambda *a:True
        node.allowed_planning_time=5.;node.num_planning_attempts=3;node.planner_id='RRTConnectkConfigDefault'
        node.end_effector_link='grasp_tcp';node.position_tolerance_m=.01
        node.use_orientation_constraint=True;node.orientation_tolerance_rad=.05
        node._add_local_base_goal_constraints=Mock();node._state_valid=lambda *a,**k:True
        node._ik_candidates=lambda *a:[]
        p=PoseStamped();p.header.frame_id='world';p.pose.orientation.w=1.
        self.assertIsNone(node._plan_pose('test',p,{}))
        node.move_group_client.send_goal_async.assert_not_called()

    def test_incomplete_cartesian_path_never_executes(self):
        node=executor();node.cartesian_client=Mock();node.end_effector_link='grasp_tcp'
        from moveit_msgs.msg import MotionPlanRequest
        request=MotionPlanRequest(group_name='arm')
        node._call_service=lambda *a,**k:SimpleNamespace(error_code=SimpleNamespace(val=1),fraction=.9)
        node._execute_preplanned=Mock()
        self.assertIsNone(node._plan_cartesian('test',PoseStamped(),{},request))
        node._execute_preplanned.assert_not_called()

    def test_long_trajectory_is_one_transaction_with_original_timing(self):
        node=executor();node._arduino_result_lock=threading.Lock();node._arduino_done_event=threading.Event()
        node._pending_joint_state_lock=threading.Lock();node.arduino_cmd_pub=Mock();node.stop_pub=Mock()
        raw=[[0.0]*8 for _ in range(135)]
        points=['0 0 0 0 0 0 0 0 0 0 '+str(i*100) for i in range(135)]
        node._wait_for_arduino_done=Mock()
        node._send_trajectory_to_arduino('long',raw,points)
        node.arduino_cmd_pub.publish.assert_called_once()
        sent=node.arduino_cmd_pub.publish.call_args.args[0].data
        self.assertTrue(sent.startswith('STREAM '))
        self.assertEqual(sent.split(' ',2)[2].split(','),points)
        self.assertEqual(validate_command('CMD '+sent[7:],streaming=True)[1],13.4)
        node._wait_for_arduino_done.assert_not_called()

    def test_transit_endpoint_accepts_reached_state_but_contact_remains_precise(self):
        node=executor();now=time.monotonic();node._command_started=now-2
        node._pending_joint_state=[0.0]*8
        node._joint_history=[(now-d,state()) for d in (.6,.4,.2,0)]
        for _,js in node._joint_history:js.position[7]=math.radians(.8)
        node._endpoint_profile='transit'
        self.assertTrue(node._measured_goal_reached())
        node._endpoint_profile='precise'
        self.assertFalse(node._measured_goal_reached())

    def test_dense_transit_stream_rejected_before_any_serial_write(self):
        bridge=ArduinoBridge.__new__(ArduinoBridge);bridge.active_id=42;bridge.ser=Mock()
        bridge.stream=dict(points=[f'0 0 0 0 0 0 0 0 0 0 {i}' for i in range(60)],
                           offset=0,sequence=0,transit=True)
        with self.assertRaisesRegex(ValueError,'too dense'):
            bridge._refill_stream(0,45)
        bridge.ser.write.assert_not_called()

    def test_missing_byte_ack_stops_without_sending_more_trajectory(self):
        bridge=ArduinoBridge.__new__(ArduinoBridge);bridge.ser=Mock();bridge._fail=Mock()
        bridge.tx=dict(payload=b'x'*100,sent=32,acked=0,last_progress=10.0)
        with patch('arduino_reader.time.monotonic',return_value=11.0):bridge._pump_tx()
        bridge.ser.write.assert_called_once_with(b'\nSTOP\n')
        bridge._fail.assert_called_once_with('RX_ACK_TIMEOUT')

    def test_heartbeat_never_interleaves_with_incomplete_packet(self):
        bridge=ArduinoBridge.__new__(ArduinoBridge);bridge.ser=Mock()
        bridge.ser.in_waiting=0;bridge.ser.read.return_value=b''
        bridge.compatible=True;bridge.active_id=42;bridge.deadline=200
        bridge.last_feedback=100;bridge.last_ping=0;bridge.rx=bytearray()
        bridge.serial_bytes_received=0
        bridge.tx=dict(payload=b'x'*100,sent=32,acked=0,last_progress=100)
        with patch('arduino_reader.time.monotonic',return_value=100):bridge._read_loop()
        bridge.ser.write.assert_not_called()

    def test_stream_credit_sequence_and_final_packet(self):
        bridge=ArduinoBridge.__new__(ArduinoBridge)
        bridge.active_id=42;bridge.ser=Mock()
        bridge.ser.write.side_effect=lambda b:len(b)
        points=[f'0 0 0 0 0 0 0 0 0 0 {i*100}' for i in range(60)]
        bridge.stream=dict(points=points,offset=0,sequence=0)
        with patch('arduino_reader.time.sleep'):
            bridge._refill_stream(0,45)
            self.assertEqual(bridge.stream['offset'],24)
            calls=bridge.ser.write.call_count
            bridge._refill_stream(0,45) # stale credit cannot refill
            bridge._refill_stream(1,11) # insufficient capacity
            self.assertEqual(bridge.ser.write.call_count,calls)
            def drain():
                while bridge.tx is not None:
                    sent=bridge.tx['sent']
                    before=bridge.ser.write.call_count
                    bridge._pump_tx()
                    self.assertEqual(before,bridge.ser.write.call_count) # waits for consumed bytes
                    bridge._handle_line(f'RX4 {sent}')
                    bridge._pump_tx()
            drain()
            for seq in (1,2,3):
                bridge._refill_stream(seq,12)
                drain()
        wire=b''.join(c.args[0] for c in bridge.ser.write.call_args_list).decode()
        packets=wire.strip().split('\n')
        self.assertEqual(len(packets),4)
        import binascii
        recovered=[]
        for i,packet in enumerate(packets):
            prefix,cid,crc,body=packet.split(' ',3)
            self.assertEqual(int(crc,16),binascii.crc_hqx(body.encode(),0xffff))
            sequence,final,queue=body.split(' ',2)
            self.assertEqual(int(sequence),i)
            self.assertEqual(int(final),int(i==3))
            recovered.extend(queue.split(','))
        self.assertEqual(recovered,points)

    def test_base_bounds_rotate_with_approach(self):
        node=executor();pose=PoseStamped();pose.pose.position.x=1.;pose.pose.position.y=2.
        pose.pose.orientation.z=math.sqrt(.5);pose.pose.orientation.w=math.sqrt(.5)
        from moveit_msgs.msg import Constraints
        constraints=Constraints()
        node._add_local_base_goal_constraints(constraints,{'base_goal_tcp_x_min_m':.5,
            'base_goal_tcp_x_max_m':.7,'base_goal_tcp_y_max_m':.2},'rotated',pose)
        values={c.joint_name:c for c in constraints.joint_constraints}
        self.assertAlmostEqual(values['base_x'].position,1.)
        self.assertAlmostEqual(values['base_y'].position,1.4)
        self.assertAlmostEqual(values['base_theta'].position,math.pi/2)

    def test_intermediate_collision_rejects_serialized_path(self):
        node=executor();node._state_valid=lambda js,*a,**k: not .009 < js.position[0] < .011
        start=[0.]*8;end=[.02]+[0.]*7
        self.assertFalse(node._validate_executed_path([start,end],{'group_name':'base'}))

    def test_theta_is_unwrapped_for_firmware(self):
        node=executor();node.latest_joint_state.position[2]=3.13
        node._trajectory_validation_error=lambda jt:None
        node.pretty_print_trajectory_waypoints=False
        trajectory=RobotTrajectory();jt=trajectory.joint_trajectory
        jt.joint_names=list(TRAJECTORY_JOINT_ORDER)
        for theta,t in [(3.13,0),(-3.13,1)]:
            point=JointTrajectoryPoint(positions=[0.,0.,theta,0.,0.,0.,0.,0.])
            point.time_from_start.sec=t;jt.points.append(point)
        _,raw,_=node._extract_trajectory_waypoints('wrap',trajectory)
        self.assertLess(abs(raw[-1][2]-raw[0][2]),.03)

    def test_default_tasks_accept_one_detection_without_stability_wait(self):
        node=executor();node._detection_lock=threading.Lock()
        node.detection_min_stable_samples=1;node.detection_stability_tolerance_m=.015
        node.detection_max_refresh_shift_m=.15
        pose=PoseStamped();pose.header.stamp=Time(nanoseconds=int(99e9)).to_msg()
        pose.pose.position.x=.85;pose.pose.orientation.w=1.
        node._detection_samples={'cup':[pose]}
        result=node._wait_for_detection('cup',.1)
        self.assertIsNotNone(result)
        self.assertEqual(result.pose.position.x,.85)
        commands=node._load_execution_map(str(ROOT/'homeplus_moveit_config/config/gesture_execution_map.yaml'))
        for name in ('pick_up_coffee','press_square_button'):
            self.assertEqual(commands[name]['detection_min_stable_samples'],1)
            for step in commands[name]['steps']:
                if step['type']=='detect':self.assertEqual(step['detection_min_stable_samples'],1)

    def test_refresh_accepts_target_beyond_old_six_cm_gate_when_disabled(self):
        node=executor();node._detection_lock=threading.Lock()
        node.detection_min_stable_samples=1;node.detection_stability_tolerance_m=.015
        node.detection_max_refresh_shift_m=0.
        pose=PoseStamped();pose.header.stamp=Time(nanoseconds=int(99e9)).to_msg()
        pose.pose.position.x=.85;pose.pose.orientation.w=1.
        reference=PoseStamped();reference.pose.position.x=.65
        node._detection_samples={'cup':[pose]}
        result=node._wait_for_detection('cup',.1,reference_pose=reference)
        self.assertIsNotNone(result)
        self.assertEqual(result.pose.position.x,.85)
        params=yaml.safe_load((ROOT/'homeplus_moveit_config/config/gesture_executor_params.yaml').read_text())
        self.assertTrue(any(v.get('ros__parameters',{}).get('detection_max_refresh_shift_m')==0
                            for v in params.values()))

    def test_shipped_execution_map_loads_through_startup_validator(self):
        commands = executor()._load_execution_map(str(
            ROOT/'homeplus_moveit_config/config/gesture_execution_map.yaml'))
        self.assertIn('pick_up_coffee', commands)
        self.assertIn('press_square_button', commands)

    def test_endpoint_profile_validation(self):
        base = dict(type='pose', position=[0.5, 0.0, 0.8], group_name='arm')
        for profile in ('precise', 'transit'):
            self.assertEqual(GestureMotionExecutor._command_config_errors(
                'test', dict(base, endpoint_profile=profile)), [])
        for profile in ('typo', None, 1, []):
            self.assertTrue(GestureMotionExecutor._command_config_errors(
                'test', dict(base, endpoint_profile=profile)))
        self.assertTrue(GestureMotionExecutor._command_config_errors(
            'test', dict(base, cartesian=True, endpoint_profile='transit')))

    def test_configuration_typo_is_not_silently_ignored(self):
        errors=GestureMotionExecutor._command_config_errors('test',{'type':'pose',
            'position':[0.,0.,1.],'minimum_tcp_clearence_m':.05})
        self.assertTrue(any('unsupported configuration' in e for e in errors))



class GeometryTests(unittest.TestCase):
    def test_preferred_arm_solution_keeps_measured_base_and_is_revalidated(self):
        from moveit_msgs.msg import Constraints
        node = executor(); node.default_group_name='arm'
        preferred = state()
        preferred.position[preferred.name.index('base_x')] = .2
        preferred.position[preferred.name.index('joint_elbow')] = .4
        node._state_valid = Mock(return_value=True)
        node._call_service = Mock()
        result = node._ik_candidates(PoseStamped(), {'group_name':'arm', '_preferred_arm_state':preferred}, Constraints())
        self.assertAlmostEqual(result[0].position[result[0].name.index('base_x')], 0.)
        self.assertAlmostEqual(result[0].position[result[0].name.index('joint_elbow')], .4)
        node._state_valid.assert_called_once()
        node._call_service.assert_not_called()
        node._state_valid.return_value=False
        self.assertEqual(node._ik_candidates(PoseStamped(), {'group_name':'arm', '_preferred_arm_state':preferred, 'ik_candidate_attempts':0}, Constraints()), [])

    def test_cartesian_failure_probe_preserves_execution_checks(self):
        import tempfile
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import GetCartesianPath
        node = executor(); node.scene_client = object(); node.cartesian_client = object()
        path = GetCartesianPath.Request(); path.avoid_collisions=True; path.group_name='arm'
        path.start_state.joint_state=state()
        failed = GetCartesianPath.Response(); failed.fraction=.28
        probe = GetCartesianPath.Response(); probe.fraction=1.
        probe.solution.joint_trajectory.joint_names=['joint_elbow']
        probe.solution.joint_trajectory.points=[JointTrajectoryPoint(positions=[.1])]
        node._call_service=Mock(side_effect=[SimpleNamespace(scene=PlanningScene()),probe])
        node._state_valid=Mock(return_value=False)
        node._execute_preplanned=Mock()
        with tempfile.TemporaryDirectory() as directory, patch('gesture_motion_executor.tempfile.mkdtemp', return_value=directory):
            self.assertIsNone(node._diagnose_cartesian_failure('test',path,failed))
            self.assertTrue((Path(directory)/'scene.cdr').exists())
        self.assertTrue(path.avoid_collisions)
        self.assertFalse(node._call_service.call_args.args[1].avoid_collisions)
        node._execute_preplanned.assert_not_called()
        node._state_valid.assert_called_once()

    def test_cup_envelope_is_offset_from_stable_surface_not_centered_on_it(self):
        from moveit_msgs.msg import AllowedCollisionMatrix
        node = executor()
        node._cup_geometry = {'stamp_ns': 0, 'radius': .065, 'height': .15,
            'source': 'configured_collision_envelope', 'center': [.88, .12, .8],
            'collision_center': [.945, .12, .7775]}
        node.scene_client = object(); node.apply_scene_client = object()
        node._call_service = Mock(side_effect=[SimpleNamespace(scene=SimpleNamespace(
            allowed_collision_matrix=AllowedCollisionMatrix())), SimpleNamespace(success=True)])
        pose = PoseStamped(); pose.pose.position.x=.89; pose.pose.position.y=.13; pose.pose.position.z=.81
        node._update_cup_scene(pose)
        center = node._cup_collision_object.primitive_poses[0].position
        self.assertAlmostEqual(center.x, .955)
        self.assertAlmostEqual(center.y, .13)
        self.assertAlmostEqual(center.z, .7875)

    def test_mobile_standoff_uses_continuous_path_not_endpoint_ik_gate(self):
        from moveit_msgs.msg import Constraints
        node = executor()
        node.end_effector_link = 'grasp_tcp'
        node.cartesian_client = object()
        node._resolve_detected_pose = lambda cfg, pose: {'position': [.7, .1, .8]}
        node._call_service = Mock(return_value=SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=1.0))
        node._plan_joint_phase = Mock(return_value=object())
        node._execute_preplanned = Mock(return_value=SimpleNamespace(success=True))
        node._wait_for_arduino_done = Mock(return_value=True)
        node._plan_pose = Mock(return_value='arm_plan')
        node._diagnose_cartesian_failure = Mock()
        pose = PoseStamped(); pose.header.frame_id = 'world'; pose.pose.orientation.w = 1.
        cfg = {'group_name': 'base_arm', 'lookahead_offsets': [[.19, 0., 0.]]}
        req = SimpleNamespace(goal_constraints=[Constraints()])
        self.assertEqual(node._prepare_mobile_standoff('test', pose, cfg, req, [state()]), 'arm_plan')
        self.assertIs(node._call_service.call_args.args[0], node.cartesian_client)
        self.assertTrue(node._call_service.call_args.args[1].avoid_collisions)
        node._call_service.return_value.fraction = .8
        node._execute_preplanned.reset_mock()
        self.assertIsNone(node._prepare_mobile_standoff('test', pose, cfg, req, [state()]))
        node._execute_preplanned.assert_not_called()

    def test_continuation_diagnostic_never_returns_unchecked_candidate(self):
        from moveit_msgs.srv import GetPositionIK
        node = executor()
        node.ik_client = Mock()
        node._call_service = Mock(return_value=SimpleNamespace(
            error_code=SimpleNamespace(val=1), solution=SimpleNamespace(joint_state=state())))
        node._state_valid = Mock(return_value=False)
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.avoid_collisions = True
        result = node._diagnose_continuation_ik(request, state(), 'test', 1)
        self.assertIsNone(result)
        self.assertTrue(request.ik_request.avoid_collisions)
        self.assertFalse(node._call_service.call_args.args[1].ik_request.avoid_collisions)
        node._state_valid.assert_called_once()

    def test_joint_noop_requires_ready_fresh_and_close_state(self):
        node = executor()
        node._require_fresh_state = Mock()
        node._hardware_ready = True
        goal = JointState(name=['joint_elbow', 'joint_shoulder'], position=[0., 0.])
        self.assertTrue(node._joint_goal_already_reached(goal))
        node._hardware_ready = False
        self.assertFalse(node._joint_goal_already_reached(goal))
        node._hardware_ready = True
        goal.position = [.02, 0.]
        self.assertFalse(node._joint_goal_already_reached(goal))
        node._require_fresh_state.side_effect = RuntimeError('stale')
        with self.assertRaises(RuntimeError):
            node._joint_goal_already_reached(goal)

    def test_surface_goal_offset_follows_view_and_preserves_height(self):
        point, direction = offset_surface_goal([1., 2., .8], [1., 1., 1.2], .03)
        np.testing.assert_allclose(point, [1., 2.03, .8])
        np.testing.assert_allclose(direction, [0., 1., 0.])
        with self.assertRaises(ValueError):
            offset_surface_goal([1., 1., .8], [1., 1., 1.2], .03)

    def test_tapered_cup_center_and_grasp_width_across_views(self):
        rng = np.random.default_rng(15)
        for angle in (-1.0, 0.0, 1.0):
            z = rng.uniform(.72, .84, 1800)
            theta = rng.uniform(angle-1.1, angle+1.1, len(z))
            radius = .025 + .22*(z-.72)
            pts = np.column_stack((1.2+radius*np.cos(theta),
                                   .2+radius*np.sin(theta), z))
            pts += rng.normal(0, .0006, pts.shape)
            details = {}
            result = fit_cup_center(pts, diagnostics=details)
            self.assertIsNotNone(result, details)
            center, enclosing_radius, height = result
            self.assertLess(np.linalg.norm(center[:2]-[1.2,.2]), .003)
            self.assertAlmostEqual(details['grasp_radius'], .025+.22*(center[2]-.72), delta=.003)
            self.assertGreaterEqual(enclosing_radius, details['grasp_radius'])
            self.assertAlmostEqual(details['taper'], .22, delta=.04)

    def test_cylinder_center_invariant_to_view(self):
        rng=np.random.default_rng(1)
        for angle in (-1.,0.,1.):
            theta=rng.uniform(angle-1.1,angle+1.1,1000)
            pts=np.column_stack((1.2+.04*np.cos(theta), .2+.04*np.sin(theta),rng.uniform(.72,.84,1000)))
            pts+=rng.normal(0,.0005,pts.shape)
            out=fit_cup_center(pts)
            self.assertIsNotNone(out)
            self.assertLess(np.linalg.norm(out[0][:2]-[1.2,.2]),.003)
            self.assertAlmostEqual(out[1],.04,delta=.003)

    def test_rejects_table_and_tiny_arc(self):
        rng=np.random.default_rng(2)
        table=np.column_stack((rng.uniform(0,1,100),rng.uniform(0,1,100),np.full(100,.72)))
        self.assertIsNone(fit_cup_center(table))
        theta=rng.uniform(-.05,.05,500)
        arc=np.column_stack((.04*np.cos(theta),.04*np.sin(theta),rng.uniform(.72,.84,500)))
        self.assertIsNone(fit_cup_center(arc))

    def test_button_keeps_pointing_ray_on_plane(self):
        x,y=np.meshgrid(np.linspace(-.015,.015,20),np.linspace(-.015,.015,20))
        pts=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.5)))
        out=fit_button_plane(pts,[.01,0,1])
        self.assertIsNotNone(out)
        np.testing.assert_allclose(out[0],[.005,0,.5],atol=1e-6)
        self.assertGreater(out[1][2],0)


class PerceptionTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('detector',ROOT/'homeplus_vision/scripts/grounding_dino_node.py')
        cls.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.module)

    def test_future_camera_frame_is_rejected_at_receipt(self):
        node = self.module.MolmoPointNode.__new__(self.module.MolmoPointNode)
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1000000000))
        log = Mock()
        node.get_logger = lambda: log
        node.bridge = Mock()
        msg = JointState()
        msg.header.stamp = Time(nanoseconds=1800000000).to_msg()
        node.image_callback(msg)
        node.bridge.imgmsg_to_cv2.assert_not_called()
        log.error.assert_called_once()

    def node(self):
        node=self.module.MolmoPointNode.__new__(self.module.MolmoPointNode)
        node._state_lock=threading.Lock();node.pause_during_motion=True
        node._robot_moving=False;node._motion_finished_stamp_ns=int(90e9)
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=int(100e9)))
        node.get_parameter=lambda key:SimpleNamespace(value={'motion_settle_sec':.7,
            'max_observation_age_sec':15.,'rgbd_max_skew_sec':.035}[key])
        node.get_logger=lambda:Mock()
        return node

    def test_motion_and_settling_invalidate_inference(self):
        node=self.node()
        self.assertFalse(node._frame_is_publishable(int(90.5e9)))
        self.assertTrue(node._frame_is_publishable(int(95e9)))
        node._robot_moving=True
        self.assertFalse(node._frame_is_publishable(int(95e9)))
        node._robot_moving=False;node._motion_finished_stamp_ns=int(99e9)
        self.assertFalse(node._frame_is_publishable(int(95e9)))

    def test_unpaired_rgb_depth_never_reaches_model(self):
        from std_msgs.msg import Header
        from sensor_msgs.msg import Image,CameraInfo
        node=self.node();node.latest_color=np.zeros((10,10,3),dtype=np.uint8)
        node.latest_color_header=Header(stamp=Time(nanoseconds=int(95e9)).to_msg())
        node._depth_frames=deque([Image(header=Header(stamp=Time(nanoseconds=int(95.1e9)).to_msg()))])
        node.camera_info=CameraInfo();node.run_model_inference=Mock()
        node.detect_loop()
        node.run_model_inference.assert_not_called()

    def test_transform_retains_acquisition_time(self):
        from geometry_msgs.msg import TransformStamped
        node=self.node();node.world_frame='world'
        camera=PoseStamped();camera.header.frame_id='camera'
        camera.header.stamp=Time(nanoseconds=int(95e9)).to_msg();camera.pose.orientation.w=1.
        tf=TransformStamped();tf.header.frame_id='world';tf.header.stamp=Time(nanoseconds=int(100e9)).to_msg()
        tf.transform.rotation.w=1.;tf.transform.translation.x=1.
        out=node._transform_pose_to_world(camera,tf)
        self.assertEqual(out.header.stamp.sec,95)
        self.assertEqual(out.pose.position.x,1.)


class ConfigTests(unittest.TestCase):
    def test_all_contact_stages_fixed_base_and_cartesian(self):
        commands=yaml.safe_load((ROOT/'homeplus_moveit_config/config/gesture_execution_map.yaml').read_text())['commands']
        for name in ('pick_up_coffee','press_square_button'):
            self.assertIn(f"task_{commands[name]['task_id']}",commands[name]['detection_topic'])
            for step in commands[name]['steps']:
                if step['name'] in ('move_to_grasp','lift','retreat_with_object','press_button','retreat_from_button'):
                    self.assertTrue(step['cartesian'])
                    self.assertIn(step['group_name'],('arm','press_arm'))
                    self.assertTrue(step['use_orientation_constraint'])


class CloudTimingTests(unittest.TestCase):
    def test_delayed_tf_retains_original_frame_and_expires_missing_tf(self):
        node = ObstacleCloudNode.__new__(ObstacleCloudNode)
        node.pending_depth = None
        node.output_frame = 'world'
        node.filter_frame = 'base_link'
        node.fallback_frame = ''
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1000000000))
        node.tf_buffer = Mock()
        node.tf_buffer.can_transform.return_value = False
        node._process_depth = Mock()
        node.get_logger = lambda: Mock()
        frame = JointState()
        frame.header.frame_id = 'camera'
        frame.header.stamp = Time(nanoseconds=900000000).to_msg()
        node.depth_callback(frame)
        node.depth_callback(JointState())
        node._process_pending_depth()
        node._process_depth.assert_not_called()
        self.assertIs(node.pending_depth[0], frame)
        node.tf_buffer.can_transform.return_value = True
        node._process_pending_depth()
        node._process_depth.assert_called_once_with(frame)
        self.assertIsNone(node.pending_depth)
        node.depth_callback(frame)
        node.tf_buffer.can_transform.return_value = False
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1600000000))
        node._process_pending_depth()
        self.assertIsNone(node.pending_depth)
        self.assertEqual(node._process_depth.call_count, 1)


class MappingReadinessTests(unittest.TestCase):
    def configured_executor(self, response_generation='8', success=True):
        node = executor()
        node.clear_map_before_pose_plan = True
        node.map_clear_timeout_sec = 0.1
        node.map_rebuild_wait_sec = 3.0
        node._map_stamp_ns = int(101e9)
        node._map_generation = 8
        response = SimpleNamespace(success=success, message=response_generation)
        future = SimpleNamespace(result=lambda: response)
        future.add_done_callback = lambda callback: callback(future)
        node.map_clear_client = Mock()
        node.map_clear_client.call_async.return_value = future
        return node

    def test_matching_generation_and_inserted_mesh_allow_planning(self):
        node = self.configured_executor()
        self.assertTrue(node._clear_and_rebuild_map('grasp', {}))

    def test_old_generation_cannot_satisfy_rebuild(self):
        node = self.configured_executor()
        node._map_generation = 7
        with patch('gesture_motion_executor.time.monotonic', side_effect=[0, 1, 4]), patch('gesture_motion_executor.time.sleep'):
            self.assertFalse(node._clear_and_rebuild_map('grasp', {}))

    def test_failed_clear_prevents_planning(self):
        node = self.configured_executor('MoveIt rejected reset', False)
        self.assertFalse(node._clear_and_rebuild_map('grasp', {}))

    def test_cloud_publication_is_not_map_readiness(self):
        node = self.configured_executor()
        node._map_stamp_ns = 0
        node._map_ready_callback(String(data='{"stamp_ns":101000000000,"masked":true}'))
        self.assertEqual(node._map_stamp_ns, 0)

    def test_empty_mesh_or_insufficient_observations_are_not_ready(self):
        node = self.configured_executor()
        node._map_stamp_ns = 0
        for frames, triangles in [(2, 100), (3, 0)]:
            node._map_ready_callback(String(data='{"stamp_ns":101000000000,"generation":8,"frames":%d,"triangles":%d}' % (frames, triangles)))
        self.assertEqual(node._map_stamp_ns, 0)
        node._map_ready_callback(String(data='{"stamp_ns":101000000000,"generation":8,"frames":3,"triangles":100}'))
        self.assertEqual(node._map_stamp_ns, int(101e9))


if __name__=='__main__':unittest.main()
