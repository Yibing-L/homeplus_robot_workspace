#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints
from moveit_msgs.msg import MoveGroupActionGoal
import sys

class HomePlusMoveitController(Node):
    def __init__(self):
        super().__init__('homeplus_moveit_controller')
        
        # Create action client for move_group
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('HomePlus MoveIt Controller initialized')
        self.get_logger().info('Waiting for move_group action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to move_group action server')

        # Define the target end-effector point here
        self.target_point = [0.5, 0.0, 0.5]  # [x, y, z] in base frame
        self.plan_and_execute_to_point()
    def plan_and_execute_to_point(self):
        self.get_logger().info(f'Planning to hardcoded end-effector point: {self.target_point}')
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"
        # Set position-only constraint
        constraints = Constraints()
        from moveit_msgs.msg import PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import PoseStamped
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base"
        pos_constraint.link_name = "Hand"  # Update to your end-effector link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
        pose = PoseStamped()
        pose.header.frame_id = "base"
        pose.pose.position.x = self.target_point[0]
        pose.pose.position.y = self.target_point[1]
        pose.pose.position.z = self.target_point[2]
        pose.pose.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def target_callback(self, msg):
        """Handle new target pose using MoveIt action"""
        self.get_logger().info('Received new target pose, planning movement...')
        
        # Create motion plan request
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        
        # Set the group name
        goal.request.group_name = "arm"
        
        # Set pose constraint
        pose_constraint = Constraints()
        # Note: This is a simplified example. In a real implementation,
        # you would need to properly set up pose constraints
        
        goal.request.goal_constraints = [pose_constraint]
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan and execute
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted, executing movement...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Movement completed successfully!')
        else:
            self.get_logger().info(f'Movement failed with error code: {result.error_code.val}')
        
        # Plan trajectory using BiRRT*
        plan = self.arm_group.plan()
        
        if plan[0]:  # plan[0] is True if planning succeeded
            self.get_logger().info('Planning succeeded')
            
            # Execute the trajectory
            self.arm_group.execute(plan[1], wait=True)  # plan[1] contains the trajectory
            
            # Clear targets
            self.arm_group.clear_pose_targets()
            
            self.get_logger().info('Execution completed')
        else:
            self.get_logger().warn('Planning failed')

    def move_to_home(self):
        """Move the robot to its home position"""
        self.arm_group.set_named_target('home')
        self.arm_group.go(wait=True)

    def set_gripper(self, width):
        """Control the gripper width"""
        # Assuming width is between 0 (closed) and 1 (open)
        width = max(0.0, min(1.0, width))  # Clamp between 0 and 1
        
        joint_values = self.gripper_group.get_current_joint_values()
        joint_values[0] = width * 1.396  # JointFingerL limit
        joint_values[1] = -width * 1.396  # JointFingerR limit
        
        self.gripper_group.go(joint_values, wait=True)

def main():
    rclpy.init()
    node = HomePlusMoveitController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
