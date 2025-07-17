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
import sys
import math

class HomePlusIKPipeline(Node):
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
        self.publish_box_obstacle()

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
        primitive.dimensions = [0.5, 0.5, 0.5]  # size: x, y, z (meters)
        box.primitives.append(primitive)
        # Define box pose
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 1.0
        pose.position.z = 0.25
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
        
        self.get_logger().info('HomePlus IK Pipeline initialized')
        self.get_logger().info('Waiting for move_group action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to move_group action server')

    def plan_trajectory_to_point(self):
        """Plan trajectory to reach the target point and output trajectory strings"""
        self.get_logger().info(f'Planning trajectory to end-effector point: {self.target_point}')
        
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "base_arm"  # Must match RViz planning group
        # Set planner_id to match config
        goal.request.planner_id = "BiRRT"
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
        
        self.get_logger().info("Using BiRRT planner with pose constraints and obstacle avoidance")
        
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

    def extract_and_output_trajectory(self, joint_trajectory):
        """Extract trajectory and output as list of configuration strings"""
        joint_names = joint_trajectory.joint_names
        points = joint_trajectory.points
        
        self.get_logger().info(f'Trajectory has {len(points)} waypoints')
        self.get_logger().info(f'Joint names: {joint_names}')
        
        # Define the order we want: base_x, base_y, base_theta, JointFrame, JointTelescope, JointArm1, JointArm2, JointWrist, JointHand
        desired_joints = [
            'base_x', 'base_y', 'base_theta', 
            'JointFrame', 'JointTelescope', 
            'JointArm1', 'JointArm2', 'JointWrist', 'JointHand'
        ]
        
        # Create mapping from joint name to index
        joint_indices = {}
        for i, name in enumerate(joint_names):
            joint_indices[name] = i
        
        trajectory_strings = []
        
        for i, point in enumerate(points):
            # Extract values for desired joints in order
            values = []
            for joint_name in desired_joints:
                if joint_name in joint_indices:
                    idx = joint_indices[joint_name]
                    values.append(f"{point.positions[idx]:.6f}")
                else:
                    # If joint not in trajectory, use 0.0 as default
                    values.append("0.000000")
            
            config_string = " ".join(values)
            trajectory_strings.append(config_string)
            
            if i < 3 or i >= len(points) - 3 or i == len(points) // 2:
                self.get_logger().info(f'Waypoint {i}: {config_string}')
        
        self.get_logger().info('\n=== COMPLETE TRAJECTORY ===')
        for i, config_str in enumerate(trajectory_strings):
            print(f"Waypoint {i}: {config_str}")
        
        self.get_logger().info(f'\n=== TRAJECTORY SUMMARY ===')
        self.get_logger().info(f'Total waypoints: {len(trajectory_strings)}')
        self.get_logger().info(f'Joint order: {" ".join(desired_joints)}')
        self.get_logger().info('=== END TRAJECTORY ===\n')
        
        return trajectory_strings

    def set_target_point(self, x, y, z):
        """Set a new target point and replan"""
        self.target_point = [x, y, z]
        self.get_logger().info(f'New target point set: {self.target_point}')
        self.plan_trajectory_to_point()

def main():
    rclpy.init()
    
    node = HomePlusIKPipeline()
    node.set_target_point(0.0, 4.2, 0.4)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
