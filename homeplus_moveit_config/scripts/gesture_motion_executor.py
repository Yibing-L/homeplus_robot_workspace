#!/usr/bin/env python3

import math
from copy import deepcopy
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from homeplus_interfaces.msg import GestureCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32, String


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class GestureMotionExecutor(Node):
    def __init__(self):
        super().__init__("gesture_motion_executor")

        self.declare_parameter("execution_map_path", "")
        self.declare_parameter("command_topic", "/gesture_control/command")
        self.declare_parameter("status_topic", "/gesture_control/executor_status")
        self.declare_parameter("target_pose_topic", "/gesture_control/target_pose")
        self.declare_parameter("joint_target_topic", "/gesture_control/joint_target")
        self.declare_parameter("gripper_target_topic", "/gesture_control/gripper_target")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("enable_moveit_planning", False)
        self.declare_parameter("move_group_action_name", "/move_action")
        self.declare_parameter("default_group_name", "arm")
        self.declare_parameter("end_effector_link", "Hand")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("planner_id", "BiTRRTkConfigDefault")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("position_tolerance_m", 0.05)
        self.declare_parameter("orientation_tolerance_rad", 0.25)
        self.declare_parameter("joint_tolerance", 0.02)
        self.declare_parameter("workspace_bounds", [-2.0, -2.0, -1.0, 2.0, 2.0, 2.0])

        execution_map_path = str(self.get_parameter("execution_map_path").value)
        if not execution_map_path:
            raise ValueError("Parameter 'execution_map_path' is required.")
        if not Path(execution_map_path).is_file():
            raise FileNotFoundError(f"Execution map not found: {execution_map_path}")

        self.commands = self._load_execution_map(execution_map_path)
        self.enable_moveit_planning = bool(self.get_parameter("enable_moveit_planning").value)
        self.default_group_name = str(self.get_parameter("default_group_name").value)
        self.end_effector_link = str(self.get_parameter("end_effector_link").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.orientation_tolerance_rad = float(self.get_parameter("orientation_tolerance_rad").value)
        self.joint_tolerance = float(self.get_parameter("joint_tolerance").value)
        self.workspace_bounds = list(self.get_parameter("workspace_bounds").value)

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            10,
        )
        self.joint_target_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_target_topic").value),
            10,
        )
        self.gripper_target_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("gripper_target_topic").value),
            10,
        )

        self.create_subscription(
            GestureCommand,
            str(self.get_parameter("command_topic").value),
            self.command_callback,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.joint_states_callback,
            10,
        )

        self.latest_joint_state: Optional[JointState] = None
        self.plan_in_flight = False

        self.move_group_client = None
        if self.enable_moveit_planning:
            self.move_group_client = ActionClient(
                self,
                MoveGroup,
                str(self.get_parameter("move_group_action_name").value),
            )
            self.get_logger().info("MoveIt planning is enabled for gesture execution.")

        self.get_logger().info(
            f"Gesture motion executor ready with {len(self.commands)} commands from {execution_map_path}"
        )

    def _load_execution_map(self, path: str) -> Dict[str, Dict]:
        with open(path, "r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        commands = payload.get("commands", {})
        if not commands:
            raise ValueError(f"No commands found in {path}")
        return commands

    def joint_states_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def command_callback(self, msg: GestureCommand) -> None:
        command_name = str(msg.command_name)
        command_cfg = self.commands.get(command_name)
        if command_cfg is None:
            self._publish_status(f"No execution configured for command {command_name}")
            self.get_logger().warn(f"No execution configured for command {command_name}")
            return

        action_type = str(command_cfg.get("type", msg.command_type or "command"))
        self._publish_status(
            f"Executing command={command_name} type={action_type} "
            f"gesture={msg.gesture_id} conf={msg.confidence:.2f}"
        )

        if action_type == "pose":
            pose_msg = self._build_pose_message(command_cfg)
            self.target_pose_pub.publish(pose_msg)
            if self.enable_moveit_planning:
                self._plan_pose(command_name, pose_msg, command_cfg)
        elif action_type == "joint_state":
            joint_msg = self._build_joint_state_message(command_cfg)
            self.joint_target_pub.publish(joint_msg)
            if self.enable_moveit_planning:
                self._plan_joint_state(command_name, joint_msg, command_cfg)
        elif action_type == "gripper":
            self.gripper_target_pub.publish(Float32(data=float(command_cfg["value"])))
        elif action_type == "command":
            pass
        else:
            self.get_logger().warn(f"Unsupported execution type '{action_type}' for {command_name}")

        self.get_logger().info(f"Executed command '{command_name}'")

    def _build_pose_message(self, command_cfg: Dict) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = str(command_cfg.get("frame_id", self.base_frame))

        position = command_cfg.get("position", [0.0, 0.0, 0.0])
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        orientation = command_cfg.get("orientation")
        if orientation is not None:
            pose_msg.pose.orientation.x = float(orientation[0])
            pose_msg.pose.orientation.y = float(orientation[1])
            pose_msg.pose.orientation.z = float(orientation[2])
            pose_msg.pose.orientation.w = float(orientation[3])
        else:
            rpy_deg = command_cfg.get("rpy_deg", [0.0, 0.0, 0.0])
            quat = quaternion_from_euler(
                math.radians(float(rpy_deg[0])),
                math.radians(float(rpy_deg[1])),
                math.radians(float(rpy_deg[2])),
            )
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
        return pose_msg

    def _build_joint_state_message(self, command_cfg: Dict) -> JointState:
        joints = command_cfg.get("joints", {})
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = list(joints.keys())
        joint_msg.position = [float(value) for value in joints.values()]
        return joint_msg

    def _plan_pose(self, name: str, pose_msg: PoseStamped, command_cfg: Dict) -> None:
        if self.move_group_client is None:
            return
        if self.plan_in_flight:
            self._publish_status(f"Skipping MoveIt request for {name}; previous request still active")
            return
        if not self.move_group_client.server_is_ready():
            self.get_logger().warn("MoveIt action server is not ready; publishing pose target only.")
            return

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = str(command_cfg.get("group_name", self.default_group_name))
        goal.request.planner_id = str(command_cfg.get("planner_id", self.planner_id))
        goal.request.allowed_planning_time = float(
            command_cfg.get("allowed_planning_time", self.allowed_planning_time)
        )
        goal.request.num_planning_attempts = int(
            command_cfg.get("num_planning_attempts", self.num_planning_attempts)
        )
        goal.request.start_state = RobotState()
        if self.latest_joint_state is not None:
            goal.request.start_state.joint_state = deepcopy(self.latest_joint_state)

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = pose_msg.header.frame_id
        position_constraint.link_name = str(command_cfg.get("link_name", self.end_effector_link))
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        tol = float(command_cfg.get("position_tolerance_m", self.position_tolerance_m))
        box.dimensions = [tol, tol, tol]
        region_pose = Pose()
        region_pose.position = pose_msg.pose.position
        region_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitives = [box]
        position_constraint.constraint_region.primitive_poses = [region_pose]
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = pose_msg.header.frame_id
        orientation_constraint.link_name = str(command_cfg.get("link_name", self.end_effector_link))
        orientation_constraint.orientation = pose_msg.pose.orientation
        orient_tol = float(command_cfg.get("orientation_tolerance_rad", self.orientation_tolerance_rad))
        orientation_constraint.absolute_x_axis_tolerance = orient_tol
        orientation_constraint.absolute_y_axis_tolerance = orient_tol
        orientation_constraint.absolute_z_axis_tolerance = orient_tol
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        goal.request.goal_constraints = [constraints]
        self._fill_workspace(goal.request, pose_msg.header.frame_id)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = bool(command_cfg.get("plan_only", self.plan_only))
        self._send_move_group_goal(name, goal)

    def _plan_joint_state(self, name: str, joint_msg: JointState, command_cfg: Dict) -> None:
        if self.move_group_client is None:
            return
        if self.plan_in_flight:
            self._publish_status(f"Skipping MoveIt request for {name}; previous request still active")
            return
        if not self.move_group_client.server_is_ready():
            self.get_logger().warn("MoveIt action server is not ready; publishing joint target only.")
            return

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = str(command_cfg.get("group_name", self.default_group_name))
        goal.request.planner_id = str(command_cfg.get("planner_id", self.planner_id))
        goal.request.allowed_planning_time = float(
            command_cfg.get("allowed_planning_time", self.allowed_planning_time)
        )
        goal.request.num_planning_attempts = int(
            command_cfg.get("num_planning_attempts", self.num_planning_attempts)
        )
        goal.request.start_state = RobotState()
        if self.latest_joint_state is not None:
            goal.request.start_state.joint_state = deepcopy(self.latest_joint_state)

        constraints = Constraints()
        tol = float(command_cfg.get("joint_tolerance", self.joint_tolerance))
        for joint_name, position in zip(joint_msg.name, joint_msg.position):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = tol
            joint_constraint.tolerance_below = tol
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal.request.goal_constraints = [constraints]
        self._fill_workspace(goal.request, str(command_cfg.get("frame_id", self.base_frame)))
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = bool(command_cfg.get("plan_only", self.plan_only))
        self._send_move_group_goal(name, goal)

    def _fill_workspace(self, request: MotionPlanRequest, frame_id: str) -> None:
        if len(self.workspace_bounds) != 6:
            return
        request.workspace_parameters.header.frame_id = frame_id
        request.workspace_parameters.min_corner.x = float(self.workspace_bounds[0])
        request.workspace_parameters.min_corner.y = float(self.workspace_bounds[1])
        request.workspace_parameters.min_corner.z = float(self.workspace_bounds[2])
        request.workspace_parameters.max_corner.x = float(self.workspace_bounds[3])
        request.workspace_parameters.max_corner.y = float(self.workspace_bounds[4])
        request.workspace_parameters.max_corner.z = float(self.workspace_bounds[5])

    def _send_move_group_goal(self, name: str, goal: MoveGroup.Goal) -> None:
        self.plan_in_flight = True
        self._publish_status(f"Sending MoveIt request for {name}")
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._goal_response_callback(name, done))

    def _goal_response_callback(self, name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.plan_in_flight = False
            self.get_logger().error(f"MoveIt request for {name} failed before acceptance: {exc}")
            self._publish_status(f"MoveIt request for {name} failed: {exc}")
            return

        if not goal_handle.accepted:
            self.plan_in_flight = False
            self.get_logger().warn(f"MoveIt request for {name} was rejected")
            self._publish_status(f"MoveIt request for {name} was rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._goal_result_callback(name, done))

    def _goal_result_callback(self, name: str, future) -> None:
        self.plan_in_flight = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"MoveIt result retrieval failed for {name}: {exc}")
            self._publish_status(f"MoveIt result retrieval failed for {name}: {exc}")
            return

        error_code = int(result.error_code.val)
        if error_code == 1:
            self.get_logger().info(f"MoveIt planning succeeded for {name}")
            self._publish_status(f"MoveIt planning succeeded for {name}")
        else:
            self.get_logger().warn(f"MoveIt planning failed for {name} with code {error_code}")
            self._publish_status(f"MoveIt planning failed for {name} with code {error_code}")

    def _publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureMotionExecutor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
