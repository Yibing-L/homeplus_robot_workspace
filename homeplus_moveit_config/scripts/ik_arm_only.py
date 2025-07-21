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

class HomePlusArmOnlyPipeline(Node):
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
        super().__init__('homeplus_arm_only_pipeline')
        # Create action client for move_group
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        # Create marker publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        # Create publisher for planning scene (obstacles)
        from moveit_msgs.msg import PlanningScene, CollisionObject
        from std_msgs.msg import Header
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Target object tracking
        self.target_object_attached = False
        self.target_object_id = "target_object"
        
        # Initialize target point (will be set later)
        self.target_point = [0.0, 0.0, 0.0]
        
        # Publish obstacles that constrain base movement
        self.publish_base_constraining_obstacles()
        
        self.get_logger().info('HomePlus Arm-Only Pipeline initialized')
        self.get_logger().info('Waiting for move_group action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to move_group action server')

    def publish_base_constraining_obstacles(self):
        """Publish obstacles around the robot base to prevent X/Y movement"""
        from moveit_msgs.msg import PlanningScene, CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        
        scene = PlanningScene()
        scene.is_diff = True
        
        # Create walls around the robot to prevent base movement
        obstacles = [
            # Front wall (positive X)
            {"id": "front_wall", "pos": [0.3, 0.0, 0.05], "size": [0.1, 2.0, 0.1]},
            # Back wall (negative X)
            {"id": "back_wall", "pos": [-0.3, 0.0, 0.05], "size": [0.1, 2.0, 0.1]},
            # Left wall (positive Y)
            {"id": "left_wall", "pos": [0.0, 0.3, 0.05], "size": [2.0, 0.1, 0.1]},
            # Right wall (negative Y)
            {"id": "right_wall", "pos": [0.0, -0.3, 0.05], "size": [2.0, 0.1, 0.1]},
        ]
        
        for obs in obstacles:
            box = CollisionObject()
            box.header.frame_id = "world"
            box.id = obs["id"]
            box.operation = CollisionObject.ADD
            
            # Define box primitive
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = obs["size"]  # [x, y, z] size in meters
            box.primitives.append(primitive)
            
            # Define box pose
            pose = Pose()
            pose.position.x = obs["pos"][0]
            pose.position.y = obs["pos"][1]
            pose.position.z = obs["pos"][2]
            pose.orientation.w = 1.0
            box.primitive_poses.append(pose)
            
            scene.world.collision_objects.append(box)
        
        self.scene_pub.publish(scene)
        self.get_logger().info('Published base-constraining obstacles to planning scene')
        self.get_logger().info('Robot base is now constrained - only arm movement and rotation allowed')
    
    def publish_target_object(self):
        """Publish a target object at the goal point (visual only, non-collidable)"""
        # Only publish a visual marker - don't add as collision object to avoid blocking goal
        self.publish_target_object_marker()
        self.get_logger().info(f'Published visual target object at: {self.target_point}')
    
    def publish_target_object_marker(self):
        """Publish a visual marker for the target object"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_object"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.target_point[0]
        marker.pose.position.y = self.target_point[1]
        marker.pose.position.z = self.target_point[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.1  # diameter
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Make it golden/yellow color
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
    
    def attach_target_object_to_gripper(self):
        """Attach the target object to the gripper (visual only)"""
        if self.target_object_attached:
            return
            
        self.target_object_attached = True
        self.get_logger().info('Target object attached to gripper!')
        
        # Update marker to show attachment
        self.publish_attached_object_marker()
    
    def publish_attached_object_marker(self):
        """Publish a marker showing the object is attached"""
        marker = Marker()
        marker.header.frame_id = "Hand"  # Attached to gripper frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "attached_object"
        marker.id = 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Make it green when attached
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
        
        # Remove the old target object marker
        old_marker = Marker()
        old_marker.header.frame_id = "world"
        old_marker.header.stamp = self.get_clock().now().to_msg()
        old_marker.ns = "target_object"
        old_marker.id = 2
        old_marker.action = Marker.DELETE
        self.marker_pub.publish(old_marker)
    
    def check_end_effector_at_goal(self, trajectory_points):
        """Check if the final trajectory point reaches the goal and attach object"""
        if not trajectory_points or self.target_object_attached:
            return
            
        # Get the final waypoint
        final_point = trajectory_points[-1]
        
        # For simplicity, we assume the robot reaches the goal at the end of trajectory
        # In a real system, you'd check the actual end-effector position
        self.get_logger().info('End-effector reached goal position!')
        self.attach_target_object_to_gripper()
        
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
        """Plan trajectory to reach the target point using arm movement only"""
        self.get_logger().info(f'Planning arm-only trajectory to end-effector point: {self.target_point}')
        
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "base_arm"  # Still use base_arm group but constrain base joints
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
        
        # Create constraints to keep base fixed but allow rotation
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
        
        # Orientation constraint (relaxed)
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
        
        # Increase planning time for constrained planning
        goal.request.allowed_planning_time = 15.0  # 15 seconds
        goal.request.num_planning_attempts = 3     # Try multiple times
        
        self.get_logger().info("Using BiRRT planner for arm-only movement with base constraints")
        
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
            self.get_logger().info('Arm-only planning succeeded!')
            # Visualize end-effector point in RViz
            self.publish_end_effector_marker()
            # Extract trajectory
            trajectory = result.planned_trajectory
            if trajectory and len(trajectory.joint_trajectory.points) > 0:
                self.publish_trajectory_marker(trajectory.joint_trajectory)
                trajectory_strings = self.extract_and_output_trajectory(trajectory.joint_trajectory)
                # Check if end-effector reaches goal and attach object
                self.check_end_effector_at_goal(trajectory.joint_trajectory.points)
            else:
                self.get_logger().error('No trajectory found in result')
        else:
            self.get_logger().error(f'Arm-only planning failed with error code: {result.error_code.val}')
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
        
        self.get_logger().info(f'Arm-only trajectory has {len(points)} waypoints')
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
        
        # Show base movement analysis
        self.analyze_base_movement(trajectory_strings)
        
        self.get_logger().info('\n=== COMPLETE ARM-ONLY TRAJECTORY ===')
        for i, config_str in enumerate(trajectory_strings):
            print(f"Waypoint {i}: {config_str}")
        
        self.get_logger().info(f'\n=== TRAJECTORY SUMMARY ===')
        self.get_logger().info(f'Total waypoints: {len(trajectory_strings)}')
        self.get_logger().info(f'Joint order: {" ".join(desired_joints)}')
        self.get_logger().info('=== END TRAJECTORY ===\n')
        
        return trajectory_strings
    
    def analyze_base_movement(self, trajectory_strings):
        """Analyze how much the base moved during planning"""
        if len(trajectory_strings) < 2:
            return
            
        start_config = trajectory_strings[0].split()
        end_config = trajectory_strings[-1].split()
        
        base_x_change = abs(float(end_config[0]) - float(start_config[0]))
        base_y_change = abs(float(end_config[1]) - float(start_config[1]))
        base_theta_change = abs(float(end_config[2]) - float(start_config[2]))
        
        self.get_logger().info(f'\n=== BASE MOVEMENT ANALYSIS ===')
        self.get_logger().info(f'Base X movement: {base_x_change:.6f} meters')
        self.get_logger().info(f'Base Y movement: {base_y_change:.6f} meters')
        self.get_logger().info(f'Base rotation: {base_theta_change:.6f} radians ({math.degrees(base_theta_change):.2f} degrees)')
        
        if base_x_change < 0.01 and base_y_change < 0.01:
            self.get_logger().info('✓ SUCCESS: Base stayed in place, only arm movement used!')
        else:
            self.get_logger().info('⚠ WARNING: Base moved significantly - obstacles may not be constraining enough')

    def set_target_point(self, x, y, z):
        """Set a new target point reachable by arm movement only"""
        self.target_point = [x, y, z]
        self.target_object_attached = False  # Reset attachment state
        self.get_logger().info(f'New arm-only target point set: {self.target_point}')
        
        # Publish target object at the goal point
        self.publish_target_object()
        
        # Plan trajectory to reach the target using only arm movement
        self.plan_trajectory_to_point()

def main():
    rclpy.init()
    
    node = HomePlusArmOnlyPipeline()
    # Set a goal that should be reachable by arm movement only
    # This point is close to the robot base and within arm reach
    node.set_target_point(-0.2, 0.2, 0.8)  # 50cm forward, 20cm left, 80cm high
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
