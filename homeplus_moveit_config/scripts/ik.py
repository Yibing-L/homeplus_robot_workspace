#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints
from moveit_msgs.msg import PositionConstraint, OrientationConstraint, Constraints
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
import sys
import math
import csv
import os
import serial
import time
from datetime import datetime

class HomePlusIKPipeline(Node):
    def set_target_point(self, x, y, z):
        """Set a new target point and replan (no orientation)"""
        self.target_point = [x, y, z]
        self.get_logger().info(f'New target point set: {self.target_point}')
        self.plan_trajectory_to_point()

    def set_target_point_rpy(self, x, y, z, roll, pitch, yaw):
        """Set a new target point with RPY orientation and replan"""
        self.target_point = [x, y, z]
        self.target_rpy = [roll, pitch, yaw]
        self.get_logger().info(f'New target point set: {self.target_point}')
        self.get_logger().info(f'Target orientation (RPY): {self.target_rpy}')
        self.plan_trajectory_to_point_rpy()

    # Position-only and RPY orientation planning methods available
    def publish_trajectory_marker(self, joint_trajectory):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for point in joint_trajectory.points:
            pt = Point()
            pt.x = point.positions[0]  # base_x
            pt.y = point.positions[1]  # base_y
            pt.z = point.positions[2]  # base_theta (if z is not height, adjust accordingly)
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    def __init__(self):
        super().__init__('homeplus_ik_pipeline')
        # Create action client for move_group
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        # Create marker publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        # Create publisher for planning scene (obstacles)
        from moveit_msgs.msg import PlanningScene, CollisionObject
        from std_msgs.msg import Header
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Arduino communication setup
        self.arduino_port = '/dev/ttyUSB0'  # Default port, can be changed
        self.arduino_baud = 9600
        self.arduino_serial = None
        
        # CSV file setup
        self.csv_filename = None
        
        self.publish_box_obstacle()  # Commented out: disable publishing the obstacle
        
        self.get_logger().info('HomePlus IK Pipeline initialized')
        self.get_logger().info('Waiting for move_action action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to move_action action server')

    def publish_box_obstacle(self):
        """Publish a box obstacle to the planning scene"""
        from moveit_msgs.msg import PlanningScene, CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        import uuid
        scene = PlanningScene()
        scene.is_diff = True
        box = CollisionObject()
        box.header.frame_id = "world"
        box.id = "box_obstacle"
        box.operation = CollisionObject.ADD
        # Define box primitive
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [1.6, 1.6, 0.7]  # size: x, y, z (meters)
        box.primitives.append(primitive)
        # Define box pose
        pose = Pose()
        pose.position.x = 1.5
        pose.position.y = 0.0
        pose.position.z = 0.35
        pose.orientation.w = 1.0
        box.primitive_poses.append(pose)
        scene.world.collision_objects.append(box)
        self.scene_pub.publish(scene)
        self.get_logger().info('Published box obstacle to planning scene')
        
    def publish_end_effector_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "end_effector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.target_point[0]
        marker.pose.position.y = self.target_point[1]
        marker.pose.position.z = self.target_point[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published goal marker at: {self.target_point}')

    def plan_trajectory_to_point(self):
        """Plan trajectory to reach the target point and output trajectory strings"""
        self.get_logger().info(f'Planning trajectory to end-effector point: {self.target_point}')
        
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"  # Must match RViz planning group
        # Set planner_id to match config
        goal.request.planner_id = "BiTRRTkConfigDefault"
        # Set start state to default robot state from planning scene
        from moveit_msgs.msg import RobotState
        from moveit_msgs.msg import PlanningScene
        import rclpy.task
        # Get the current planning scene
        planning_scene_msg = None
        def planning_scene_callback(msg):
            nonlocal planning_scene_msg
            planning_scene_msg = msg
        planning_scene_sub = self.create_subscription(PlanningScene, '/planning_scene', planning_scene_callback, 1)
        # Wait for a planning scene message
        timeout = self.get_clock().now().nanoseconds + int(2e9)  # 2 seconds
        while planning_scene_msg is None and self.get_clock().now().nanoseconds < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(planning_scene_sub)
        if planning_scene_msg and planning_scene_msg.robot_state:
            goal.request.start_state = planning_scene_msg.robot_state
        else:
            goal.request.start_state = RobotState()
        
        # Initialize joint state with Arduino's actual initial positions to avoid empty JointState error
        from sensor_msgs.msg import JointState
        js = JointState()
        js.name = ['base_x', 'base_y', 'base_theta', 'JointFingerL', 'JointHand', 'JointWrist', 'JointArm2', 'JointArm1', 'JointTelescope', 'JointFrame']
        
        # Convert Arduino initial servo positions to radians/meters to match MoveIt format
        # Arduino setup(): hand=1000, wrist=900, elbow=1500, shoulder=2100, la=450, frame=0
        # Using Arduino's map() function in reverse to get degrees, then convert to radians
        
        # hand_target = map(hand_, -90, 75, 925, 1110) -> 1000 maps to ~-45°
        initial_hand_deg = -45.0  # Arduino initial hand position in degrees
        initial_hand_rad = initial_hand_deg * 3.14159 / 180.0
        
        # wrist_target = map(wrist_, -80, 340, 545, 2400) -> 900 maps to ~-60°  
        initial_wrist_deg = -60.0  # Arduino initial wrist position in degrees
        initial_wrist_rad = initial_wrist_deg * 3.14159 / 180.0
        
        # elbow_target = map(elbow_, -73, 168, 2400, 550) -> 1500 maps to ~47°
        initial_elbow_deg = 47.0  # Arduino initial elbow position in degrees  
        initial_elbow_rad = initial_elbow_deg * 3.14159 / 180.0
        
        # shoulder_target = map(shoulder_, 0, 185, 2400, 550) -> 2100 maps to ~0°
        initial_shoulder_deg = 0.0  # Arduino initial shoulder position in degrees
        initial_shoulder_rad = initial_shoulder_deg * 3.14159 / 180.0
        
        # Linear actuators: la=450mm=0.45m, frame=0mm=0.0m
        initial_telescope = -0.43  # JointTelescope: Arduino la starts at 450mm, joint range is -0.43 to 0
        initial_frame = 0.0        # JointFrame: Arduino frame starts at 0mm
        
        # Gripper: Arduino grip starts closed (2100 -> 0%), convert to radians
        initial_grip = 100.0  # Closed gripper position
        
        js.position = [
            0.0,                    # base_x: start at origin
            0.0,                    # base_y: start at origin  
            0.0,                    # base_theta: start at 0 rotation
            initial_grip,           # JointFingerL: Arduino's initial grip (closed), JointFingerR mimics this
            initial_hand_rad,       # JointHand: Arduino's initial hand position (~-45°)
            initial_wrist_rad,      # JointWrist: Arduino's initial wrist position (~-60°)
            initial_elbow_rad,      # JointArm2: Arduino's initial elbow position (~47°)
            initial_shoulder_rad,   # JointArm1: Arduino's initial shoulder position (~0°)
            initial_telescope,      # JointTelescope: Arduino's initial LA position (450mm)
            initial_frame           # JointFrame: Arduino's initial frame position (0mm)
        ]
        js.header.stamp = self.get_clock().now().to_msg()
        goal.request.start_state.joint_state = js

        
        # Create proper pose goal using separate position and orientation constraints
        constraints = Constraints()
        
        # Position constraint for end-effector
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "Hand"  # End-effector link
        # Define a small box region at the goal point
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]  # 5cm cube
        region_pose = Pose()
        region_pose.position.x = self.target_point[0]
        region_pose.position.y = self.target_point[1]
        region_pose.position.z = self.target_point[2]
        region_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitives = [box]
        position_constraint.constraint_region.primitive_poses = [region_pose]
        position_constraint.weight = 1.0
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "Hand"
        orientation_constraint.orientation.w = 1.0  # Default orientation
        orientation_constraint.absolute_x_axis_tolerance = 6.28  # ~360 degrees
        orientation_constraint.absolute_y_axis_tolerance = 6.28
        orientation_constraint.absolute_z_axis_tolerance = 6.28
        orientation_constraint.weight = 0.01  # Very low weight
        
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints = [constraints]
        
        # Set workspace bounds
        goal.request.workspace_parameters.header.frame_id = "world"
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0
        
        # Set planning options - PLAN ONLY, no execution
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True  # Only plan, don't execute
        
        # Increase planning time and attempts for obstacle avoidance
        goal.request.allowed_planning_time = 10.0  # 10 seconds
        goal.request.num_planning_attempts = 5     # Try 5 times
        
        # Send planning request
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def plan_trajectory_to_point_rpy(self):
        """Plan trajectory to reach the target point with specific RPY orientation"""
        self.get_logger().info(f'Planning trajectory to end-effector point: {self.target_point}')
        self.get_logger().info(f'Target orientation (RPY): {self.target_rpy}')
        
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"  # Must match RViz planning group
        # Set planner_id to match config
        goal.request.planner_id = "BiTRRTkConfigDefault"
        # Set start state to default robot state from planning scene
        from moveit_msgs.msg import RobotState
        from moveit_msgs.msg import PlanningScene
        import rclpy.task
        # Get the current planning scene
        planning_scene_msg = None
        def planning_scene_callback(msg):
            nonlocal planning_scene_msg
            planning_scene_msg = msg
        planning_scene_sub = self.create_subscription(PlanningScene, '/planning_scene', planning_scene_callback, 1)
        # Wait for a planning scene message
        timeout = self.get_clock().now().nanoseconds + int(2e9)  # 2 seconds
        while planning_scene_msg is None and self.get_clock().now().nanoseconds < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(planning_scene_sub)
        if planning_scene_msg and planning_scene_msg.robot_state:
            goal.request.start_state = planning_scene_msg.robot_state
        else:
            goal.request.start_state = RobotState()
        
        # Initialize joint state with Arduino's actual initial positions to avoid empty JointState error
        from sensor_msgs.msg import JointState
        js = JointState()
        js.name = ['base_x', 'base_y', 'base_theta', 'JointFingerL', 'JointHand', 'JointWrist', 'JointArm2', 'JointArm1', 'JointTelescope', 'JointFrame']
        
        # Convert Arduino initial servo positions to radians/meters to match MoveIt format
        # Arduino setup(): hand=1000, wrist=900, elbow=1500, shoulder=2100, la=450, frame=0
        # Using Arduino's map() function in reverse to get degrees, then convert to radians
        
        # hand_target = map(hand_, -90, 75, 925, 1110) -> 1000 maps to ~-45°
        initial_hand_deg = -45.0  # Arduino initial hand position in degrees
        initial_hand_rad = initial_hand_deg * 3.14159 / 180.0
        
        # wrist_target = map(wrist_, -80, 340, 545, 2400) -> 900 maps to ~-60°  
        initial_wrist_deg = -60.0  # Arduino initial wrist position in degrees
        initial_wrist_rad = initial_wrist_deg * 3.14159 / 180.0
        
        # elbow_target = map(elbow_, -73, 168, 2400, 550) -> 1500 maps to ~47°
        initial_elbow_deg = 0.0  # Arduino initial elbow position in degrees  
        initial_elbow_rad = initial_elbow_deg * 3.14159 / 180.0
        
        # shoulder_target = map(shoulder_, 0, 185, 2400, 550) -> 2100 maps to ~0°
        initial_shoulder_deg = 30.0  # Arduino initial shoulder position in degrees
        initial_shoulder_rad = initial_shoulder_deg * 3.14159 / 180.0
        
        # Linear actuators: la=450mm=0.45m, frame=0mm=0.0m
        initial_telescope = -0.43  # JointTelescope: Arduino la starts at 450mm, joint range is -0.43 to 0
        initial_frame = 0.0        # JointFrame: Arduino frame starts at 0mm
        
        # Gripper: Arduino grip starts closed (2100 -> 0%), convert to radians
        initial_grip = 100.0  # Closed gripper position
        
        js.position = [
            0.0,                    # base_x: start at origin
            0.0,                    # base_y: start at origin  
            0.0,                    # base_theta: start at 0 rotation
            initial_grip,           # JointFingerL: Arduino's initial grip (closed), JointFingerR mimics this
            initial_hand_rad,       # JointHand: Arduino's initial hand position (~-45°)
            initial_wrist_rad,      # JointWrist: Arduino's initial wrist position (~-60°)
            initial_elbow_rad,      # JointArm2: Arduino's initial elbow position (~47°)
            initial_shoulder_rad,   # JointArm1: Arduino's initial shoulder position (~0°)
            initial_telescope,      # JointTelescope: Arduino's initial LA position (450mm)
            initial_frame           # JointFrame: Arduino's initial frame position (0mm)
        ]
        js.header.stamp = self.get_clock().now().to_msg()
        goal.request.start_state.joint_state = js
        
        # Create proper pose goal using separate position and orientation constraints
        constraints = Constraints()
        
        # Position constraint for end-effector
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "Hand"  # End-effector link
        # Define a small box region at the goal point
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]  # 5cm cube
        region_pose = Pose()
        region_pose.position.x = self.target_point[0]
        region_pose.position.y = self.target_point[1]
        region_pose.position.z = self.target_point[2]
        region_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitives = [box]
        position_constraint.constraint_region.primitive_poses = [region_pose]
        position_constraint.weight = 1.0
        
        # Orientation constraint with specific RPY
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "Hand"
        
        # Convert RPY to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(self.target_rpy[0], self.target_rpy[1], self.target_rpy[2])
        orientation_constraint.orientation.x = quat[0]
        orientation_constraint.orientation.y = quat[1]
        orientation_constraint.orientation.z = quat[2]
        orientation_constraint.orientation.w = quat[3]
        
        # Set tight orientation tolerances for precise orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1  # ~5.7 degrees
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0  # High weight for orientation
        
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints = [constraints]
        
        # Set workspace bounds
        goal.request.workspace_parameters.header.frame_id = "world"
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0
        
        # Set planning options - PLAN ONLY, no execution
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True  # Only plan, don't execute
        
        # Increase planning time and attempts for obstacle avoidance with orientation
        goal.request.allowed_planning_time = 15.0  # 15 seconds for RPY planning
        goal.request.num_planning_attempts = 10    # More attempts for orientation planning
        
        # Send planning request
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle planning goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected')
            return
        
        self.get_logger().info('Planning goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Process planning result and extract trajectory"""
        result = future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Planning succeeded!')
            # Visualize end-effector point in RViz
            self.publish_end_effector_marker()
            # Extract trajectory
            trajectory = result.planned_trajectory
            if trajectory and len(trajectory.joint_trajectory.points) > 0:
                self.publish_trajectory_marker(trajectory.joint_trajectory)
                self.extract_and_output_trajectory(trajectory.joint_trajectory)
            else:
                self.get_logger().error('No trajectory found in result')
        else:
            self.get_logger().error(f'Planning failed with error code: {result.error_code.val}')
            error_messages = {
                -1: "FAILURE",
                -2: "PLANNING_FAILED", 
                -3: "INVALID_MOTION_PLAN",
                -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                -5: "CONTROL_FAILED",
                -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                -7: "TIMED_OUT",
                -8: "PREEMPTED",
                -10: "START_STATE_IN_COLLISION",
                -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                -12: "GOAL_IN_COLLISION",
                -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                -14: "GOAL_CONSTRAINTS_VIOLATED",
                -15: "INVALID_GROUP_NAME",
                -16: "INVALID_GOAL_CONSTRAINTS",
                -17: "INVALID_ROBOT_STATE",
                -18: "INVALID_LINK_NAME",
                -19: "INVALID_OBJECT_NAME",
                -20: "FRAME_TRANSFORM_FAILURE",
                -21: "COLLISION_CHECKING_UNAVAILABLE",
                -22: "ROBOT_STATE_STALE",
                -23: "SENSOR_INFO_STALE",
                -24: "NO_IK_SOLUTION"
            }
            error_msg = error_messages.get(result.error_code.val, f"Unknown error: {result.error_code.val}")
            self.get_logger().error(f"Error details: {error_msg}")

    def setup_arduino_connection(self, port='/dev/ttyUSB0', baud=9600):
        """Setup Arduino serial connection"""
        try:
            self.arduino_port = port
            self.arduino_baud = baud
            self.arduino_serial = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Arduino connected on {port} at {baud} baud')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.arduino_serial = None
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error connecting to Arduino: {e}')
            self.arduino_serial = None
            return False

    def send_trajectory_to_arduino(self, trajectory_data):
        """Send trajectory data to Arduino via serial"""
        if not self.arduino_serial or not self.arduino_serial.is_open:
            self.get_logger().warning('Arduino not connected. Attempting to connect...')
            if not self.setup_arduino_connection():
                return False

        try:
            # Format trajectory data for Arduino
            arduino_commands = []
            for waypoint_str in trajectory_data:
                # Convert MoveIt trajectory format to Arduino format
                arduino_command = self.format_waypoint_for_arduino(waypoint_str)
                arduino_commands.append(arduino_command)
            
            # Send all commands as comma-separated queue (Arduino expects this format)
            command_queue = ','.join(arduino_commands)
            self.get_logger().info(f'Sending trajectory queue with {len(arduino_commands)} waypoints')
            
            # Send the complete trajectory queue
            self.arduino_serial.write(f'{command_queue}\n'.encode())
            time.sleep(0.1)
            
            # Wait for Arduino acknowledgment
            response = self.arduino_serial.readline().decode().strip()
            if response:
                self.get_logger().info(f'Arduino response: {response}')
            
            self.get_logger().info(f'Successfully sent {len(arduino_commands)} waypoints to Arduino')
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error sending to Arduino: {e}')
            return False

    def format_waypoint_for_arduino(self, waypoint_str):
        """Convert extracted waypoint string to Arduino expected format (servo degrees)"""
        # Our format: base_x base_y base_theta JointFingerL JointHand JointWrist JointArm2 JointArm1 JointTelescope JointFrame
        # Output: x_cm, y_cm, theta_deg, grip, hand_deg, wrist_deg, elbow_deg, shoulder_deg, la_cm, frame_cm

        values = waypoint_str.split()
        if len(values) < 10:
            self.get_logger().warning(f'Insufficient values in waypoint: {waypoint_str}')
            return "0 0 0 0 0 0 0 0 0 0"

        try:
            base_x = float(values[0]) * 100  # m to cm
            base_y = float(values[1]) * 100
            base_theta = float(values[2]) * 180 / 3.14159  # rad to deg
            grip = float(values[3])  # JointFingerL: gripper value
            
            # Convert joint angles from radians to degrees, then constrain to Arduino ranges
            hand_deg = float(values[4]) * 180 / 3.14159  # JointHand
            hand_deg = max(-90.0, min(75.0, hand_deg))  # Constrain to Arduino range
            
            wrist_deg = float(values[5]) * 180 / 3.14159  # JointWrist
            wrist_deg = max(-80.0, min(340.0, wrist_deg))  # Constrain to Arduino range
            
            elbow_deg = float(values[6]) * 180 / 3.14159  # JointArm2
            elbow_deg = max(-73.0, min(168.0, elbow_deg))  # Constrain to Arduino range
            
            shoulder_deg = float(values[7]) * 180 / 3.14159  # JointArm1
            shoulder_deg = max(0.0, min(185.0, shoulder_deg))  # Constrain to Arduino range
            
            la = max(0.0, min(450.0, float(values[8]) * 100))  # JointTelescope
            frame = max(0.0, min(175.0, float(values[9]) * 100))  # JointFrame

            arduino_command = f"{base_x:.1f} {base_y:.1f} {base_theta:.1f} {grip:.1f} {hand_deg:.1f} {wrist_deg:.1f} {elbow_deg:.1f} {shoulder_deg:.1f} {la:.1f} {frame:.1f}"
            return arduino_command
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error formatting waypoint {waypoint_str}: {e}')
            return "0 0 0 0 0 0 0 0 0 0"

    def map_joint_to_percent(self, joint_value, min_val, max_val):
        """Map joint value from radians/meters to 0-100% range"""
        if max_val == min_val:
            return 0.0
        percentage = ((joint_value - min_val) / (max_val - min_val)) * 100
        return max(0.0, min(100.0, percentage))  # Clamp to 0-100%

    def save_trajectory_csv(self, trajectory_data, target_point):
        """Save trajectory data to CSV file (raw MoveIt output, no mapping)"""
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        target_str = f"x{target_point[0]:.2f}_y{target_point[1]:.2f}_z{target_point[2]:.2f}"
        self.csv_filename = f'trajectory_{target_str}_{timestamp}.csv'

        # Create CSV file path in the workspace
        csv_path = os.path.join('/home/yibing/ros2_ws', self.csv_filename)

        try:
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)

                # Write header: waypoint_id + all joint values (as in trajectory_data)
                # Assume all waypoints have the same number/order of values as produced by extract_and_output_trajectory
                n_joints = len(trajectory_data[0].split()) if trajectory_data else 0
                joint_headers = [f'joint_{i}' for i in range(n_joints)]
                header = ['waypoint_id'] + joint_headers + ['timestamp']
                writer.writerow(header)

                # Write trajectory data (raw, no mapping)
                for i, waypoint_str in enumerate(trajectory_data):
                    joint_values = waypoint_str.split()
                    row = [i] + joint_values + [timestamp]
                    writer.writerow(row)

                # Write metadata at the end
                writer.writerow([])  # Empty row
                writer.writerow(['# Metadata'])
                writer.writerow(['# Target Point:', f'{target_point[0]:.6f}', f'{target_point[1]:.6f}', f'{target_point[2]:.6f}'])
                writer.writerow(['# Total Waypoints:', len(trajectory_data)])
                writer.writerow(['# Planner Used:', 'BiTRRTkConfigDefault'])
                writer.writerow(['# Generated:', timestamp])

            self.get_logger().info(f'Trajectory saved to: {csv_path}')
            return csv_path

        except Exception as e:
            self.get_logger().error(f'Failed to save CSV file: {e}')
            return None

    def extract_and_output_trajectory(self, joint_trajectory):
        """Extract trajectory and output as list of configuration strings"""
        joint_names = joint_trajectory.joint_names
        points = joint_trajectory.points

        self.get_logger().info(f'Trajectory has {len(points)} waypoints')
        self.get_logger().info(f'Joint names: {joint_names}')


        # Arduino expects: x, y, theta, grip, hand, wrist, elbow, shoulder, la, frame
        # Map: base_x, base_y, base_theta, JointFingerL, JointHand, JointWrist, JointArm2, JointArm1, JointTelescope, JointFrame
        joint_indices = {name: i for i, name in enumerate(joint_names)}

        trajectory_strings = []
        n_points = len(points)
        for i, point in enumerate(points):
            values = []
            # base_x, base_y, base_theta
            for joint in ['base_x', 'base_y', 'base_theta']:
                idx = joint_indices.get(joint, None)
                values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")

            # JointFingerL (grip): use the actual gripper joint value
            idx = joint_indices.get('JointFingerL', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")

            # JointHand
            idx = joint_indices.get('JointHand', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")
            # JointWrist
            idx = joint_indices.get('JointWrist', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")
            # JointArm2 (elbow)
            idx = joint_indices.get('JointArm2', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")
            # JointArm1 (shoulder)
            idx = joint_indices.get('JointArm1', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")
            # JointTelescope (la)
            idx = joint_indices.get('JointTelescope', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")
            # JointFrame (frame)
            idx = joint_indices.get('JointFrame', None)
            values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")

            config_string = " ".join(values)
            trajectory_strings.append(config_string)
            if i < 3 or i >= n_points - 3 or i == n_points // 2:
                self.get_logger().info(f'Waypoint {i}: {config_string}')

        self.get_logger().info('\n=== COMPLETE TRAJECTORY ===')
        for i, config_str in enumerate(trajectory_strings):
            print(f"Waypoint {i}: {config_str}")

        self.get_logger().info(f'\n=== TRAJECTORY SUMMARY ===')
        self.get_logger().info(f'Total waypoints: {len(trajectory_strings)}')
        self.get_logger().info('Joint order: base_x base_y base_theta JointFingerL JointHand JointWrist JointArm2 JointArm1 JointTelescope JointFrame')
        self.get_logger().info('=== END TRAJECTORY ===\n')

        # Save trajectory to CSV
        csv_path = self.save_trajectory_csv(trajectory_strings, self.target_point)
        if csv_path:
            self.get_logger().info(f'CSV saved successfully: {csv_path}')

        # Save Arduino-only format CSV
        arduino_csv_path = self.save_arduino_trajectory_csv(trajectory_strings, self.target_point)
        if arduino_csv_path:
            self.get_logger().info(f'Arduino CSV saved successfully: {arduino_csv_path}')

        # Commented out: Send trajectory to Arduino
        # if self.send_trajectory_to_arduino(trajectory_strings):
        #     self.get_logger().info('Trajectory sent to Arduino successfully')
        # else:
        #     self.get_logger().warning('Failed to send trajectory to Arduino')

        return trajectory_strings

    def save_arduino_trajectory_csv(self, trajectory_data, target_point):
        """Save Arduino-formatted trajectory data to a simple CSV file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        target_str = f"x{target_point[0]:.2f}_y{target_point[1]:.2f}_z{target_point[2]:.2f}"
        arduino_csv_filename = f'arduino_trajectory_{target_str}_{timestamp}.csv'
        
        csv_path = os.path.join('/home/yibing/ros2_ws', arduino_csv_filename)
        
        try:
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write Arduino format header
                header = ['waypoint', 'x_cm', 'y_cm', 'theta_deg', 'grip_%', 'hand_%', 'wrist_%', 'elbow_%', 'shoulder_%', 'la_%', 'frame_%']
                writer.writerow(header)
                
                # Write Arduino formatted trajectory data
                for i, waypoint_str in enumerate(trajectory_data):
                    arduino_command = self.format_waypoint_for_arduino(waypoint_str)
                    arduino_values = arduino_command.split()
                    row = [i] + arduino_values
                    writer.writerow(row)
                
                # Write queue format for copy-paste to Arduino serial monitor
                writer.writerow([])
                writer.writerow(['# Arduino Queue Format (copy this line to send to Arduino):'])
                arduino_commands = []
                for waypoint_str in trajectory_data:
                    arduino_commands.append(self.format_waypoint_for_arduino(waypoint_str))
                queue_command = ','.join(arduino_commands)
                writer.writerow([queue_command])
                
            self.get_logger().info(f'Arduino trajectory saved to: {csv_path}')
            return csv_path
            
        except Exception as e:
            self.get_logger().error(f'Failed to save Arduino CSV file: {e}')
            return None

    def set_target_point(self, x, y, z):
        """Set a new target point and replan"""
        self.target_point = [x, y, z]
        self.get_logger().info(f'New target point set: {self.target_point}')
        self.plan_trajectory_to_point()

    def configure_arduino(self, port='/dev/ttyUSB0', baud=9600):
        """Configure Arduino connection parameters"""
        self.arduino_port = port
        self.arduino_baud = baud
        self.get_logger().info(f'Arduino configured: port={port}, baud={baud}')

def main():
    rclpy.init()
    
    node = HomePlusIKPipeline()
    
    # Configure Arduino (change port/baud as needed)
    # Common Arduino ports: 
    # Linux: /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyUSB1
    # Windows: COM3, COM4, COM5, etc.
    # macOS: /dev/cu.usbmodem*, /dev/cu.usbserial*
    arduino_port = '/dev/ttyUSB0'  # Change this to match your Arduino port
    arduino_baud = 9600            # Match the baud rate in your Arduino code
    
    node.configure_arduino(arduino_port, arduino_baud)
    
    # Try to establish Arduino connection (optional - will retry when sending)
    connection_success = node.setup_arduino_connection()
    if connection_success:
        node.get_logger().info('Arduino connection established')
    else:
        node.get_logger().warning('Arduino connection failed - will retry when sending trajectory')
    
    # Set target point and plan trajectory
    # Option 1: Position-only planning (no specific orientation)
    # Format: set_target_point(x, y, z)
    # node.set_target_point(0.6, 0.11212000250816345, 0.9)
    
    # Option 2: Position + RPY orientation planning
    # Format: set_target_point_rpy(x, y, z, roll, pitch, yaw)
    # Uncomment the line below to use RPY planning instead:
    node.set_target_point_rpy(0.6, 0.11212000250816345, 0.9, 0.0, 0.0, 0.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close Arduino connection if open
        if node.arduino_serial and node.arduino_serial.is_open:
            node.arduino_serial.close()
            node.get_logger().info('Arduino connection closed')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
