#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
import serial
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
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
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


BASE_FRAME = "world"
END_EFFECTOR_LINK = "palm_1"
MOVE_GROUP_NAME = "base_arm"
PLANNER_ID = "BiTRRTkConfigDefault"
TRAJECTORY_JOINT_ORDER = [
    "base_x",
    "base_y",
    "base_theta",
    "joint_grip_L",
    "joint_hand",
    "joint_wrist",
    "joint_elbow",
    "joint_shoulder",
]
DEFAULT_START_STATE = {
    "base_x": 0.0,
    "base_y": 0.0,
    "base_theta": 0.0,
    "joint_grip_L": 0.0,
    "joint_hand": 0.0,
    "joint_wrist": 0.0,
    "joint_elbow": 0.0,
    "joint_shoulder": 0.0,
}
CSV_OUTPUT_DIR = Path("~/ros2_ws/homeplus_robot_workspace/trajectory_logs").expanduser()


class HomePlusIKPipeline(Node):
    def __init__(self):
        super().__init__("homeplus_ik_pipeline")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.latest_joint_state: Optional[JointState] = None
        self.target_point: Optional[List[float]] = None
        self.target_rpy: Optional[List[float]] = None

        self.arduino_port = "/dev/ttyACM0"
        self.arduino_baud = 9600
        self.arduino_serial = None

        CSV_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        self.get_logger().info("Waiting for move_action action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to move_action action server")

    def joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def configure_arduino(self, port: str = "/dev/ttyACM0", baud: int = 9600) -> None:
        self.arduino_port = port
        self.arduino_baud = int(baud)
        self.get_logger().info(f"Arduino configured: port={self.arduino_port}, baud={self.arduino_baud}")

    def setup_arduino_connection(self) -> bool:
        try:
            self.arduino_serial = serial.Serial(self.arduino_port, self.arduino_baud, timeout=1)
            time.sleep(2.0)
            self.get_logger().info(f"Arduino connected on {self.arduino_port} at {self.arduino_baud} baud")
            return True
        except serial.SerialException as exc:
            self.get_logger().error(f"Failed to connect to Arduino: {exc}")
            self.arduino_serial = None
            return False

    def set_target_point(self, x: float, y: float, z: float) -> None:
        self.target_point = [float(x), float(y), float(z)]
        self.target_rpy = None
        self.get_logger().info(f"New target point set: {self.target_point}")
        self.plan_trajectory()

    def set_target_point_rpy(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        self.target_point = [float(x), float(y), float(z)]
        self.target_rpy = [float(roll), float(pitch), float(yaw)]
        self.get_logger().info(f"New target point set: {self.target_point}")
        self.get_logger().info(f"Target orientation (RPY rad): {self.target_rpy}")
        self.plan_trajectory()

    def build_start_state(self) -> RobotState:
        robot_state = RobotState()
        if self.latest_joint_state is not None:
            robot_state.joint_state = self.latest_joint_state
        return robot_state

    def publish_goal_marker(self) -> None:
        if self.target_point is None:
            return

        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.target_point[0]
        marker.pose.position.y = self.target_point[1]
        marker.pose.position.z = self.target_point[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def publish_trajectory_marker(self, joint_trajectory) -> None:
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        index_map = {name: idx for idx, name in enumerate(joint_trajectory.joint_names)}
        ix = index_map.get("base_x")
        iy = index_map.get("base_y")

        if ix is None or iy is None:
            return

        for point in joint_trajectory.points:
            pt = Point()
            pt.x = float(point.positions[ix])
            pt.y = float(point.positions[iy])
            pt.z = 0.0
            marker.points.append(pt)

        self.marker_pub.publish(marker)

    def build_goal_constraints(self) -> Constraints:
        if self.target_point is None:
            raise ValueError("Target point is not set")

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_FRAME
        position_constraint.link_name = END_EFFECTOR_LINK

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

        region_pose = Pose()
        region_pose.position.x = self.target_point[0]
        region_pose.position.y = self.target_point[1]
        region_pose.position.z = self.target_point[2]
        region_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives = [box]
        position_constraint.constraint_region.primitive_poses = [region_pose]
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = BASE_FRAME
        orientation_constraint.link_name = END_EFFECTOR_LINK

        if self.target_rpy is None:
            orientation_constraint.orientation.w = 1.0
            orientation_constraint.absolute_x_axis_tolerance = 2.0 * math.pi
            orientation_constraint.absolute_y_axis_tolerance = 2.0 * math.pi
            orientation_constraint.absolute_z_axis_tolerance = 2.0 * math.pi
            orientation_constraint.weight = 0.01
        else:
            quat = quaternion_from_euler(*self.target_rpy)
            orientation_constraint.orientation.x = quat[0]
            orientation_constraint.orientation.y = quat[1]
            orientation_constraint.orientation.z = quat[2]
            orientation_constraint.orientation.w = quat[3]
            orientation_constraint.absolute_x_axis_tolerance = 0.05
            orientation_constraint.absolute_y_axis_tolerance = 0.05
            orientation_constraint.absolute_z_axis_tolerance = 0.05
            orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def plan_trajectory(self) -> None:
        if self.target_point is None:
            return

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = MOVE_GROUP_NAME
        goal.request.planner_id = PLANNER_ID
        goal.request.start_state = self.build_start_state()
        goal.request.goal_constraints = [self.build_goal_constraints()]
        goal.request.workspace_parameters.header.frame_id = BASE_FRAME
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0
        goal.request.allowed_planning_time = 10.0 if self.target_rpy is None else 15.0
        goal.request.num_planning_attempts = 5 if self.target_rpy is None else 10

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True

        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Planning goal rejected")
            return

        self.get_logger().info("Planning goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        result = future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(f"Planning failed with error code: {result.error_code.val}")
            return

        self.get_logger().info("Planning succeeded")
        self.publish_goal_marker()

        trajectory = result.planned_trajectory
        if trajectory and trajectory.joint_trajectory.points:
            self.publish_trajectory_marker(trajectory.joint_trajectory)
            self.extract_and_output_trajectory(trajectory.joint_trajectory)
        else:
            self.get_logger().error("No joint trajectory found in planning result")

    def extract_and_output_trajectory(self, joint_trajectory) -> List[str]:
        joint_indices = {name: idx for idx, name in enumerate(joint_trajectory.joint_names)}
        trajectory_strings = []

        for point in joint_trajectory.points:
            values = []
            for joint_name in TRAJECTORY_JOINT_ORDER:
                idx = joint_indices.get(joint_name)
                values.append(f"{point.positions[idx]:.6f}" if idx is not None else "0.000000")
            trajectory_strings.append(" ".join(values))

        self.get_logger().info(f"Trajectory has {len(trajectory_strings)} waypoints")
        self.get_logger().info(
            "Joint order: " + " ".join(TRAJECTORY_JOINT_ORDER)
        )

        for i, config_str in enumerate(trajectory_strings[:3]):
            self.get_logger().info(f"Waypoint {i}: {config_str}")
        if len(trajectory_strings) > 3:
            self.get_logger().info(f"Final waypoint: {trajectory_strings[-1]}")

        csv_path = self.save_trajectory_csv(trajectory_strings)
        if csv_path is not None:
            self.get_logger().info(f"Saved trajectory CSV: {csv_path}")

        return trajectory_strings

    def save_trajectory_csv(self, trajectory_data: Sequence[str]) -> Optional[Path]:
        if self.target_point is None:
            return None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        target_str = f"x{self.target_point[0]:.2f}_y{self.target_point[1]:.2f}_z{self.target_point[2]:.2f}"
        csv_path = CSV_OUTPUT_DIR / f"trajectory_{target_str}_{timestamp}.csv"

        try:
            with csv_path.open("w", newline="", encoding="utf-8") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["waypoint_id", *TRAJECTORY_JOINT_ORDER, "timestamp"])
                for i, waypoint_str in enumerate(trajectory_data):
                    writer.writerow([i, *waypoint_str.split(), timestamp])
            return csv_path
        except OSError as exc:
            self.get_logger().error(f"Failed to save trajectory CSV: {exc}")
            return None


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan a MoveIt trajectory for the current HomePlus arm")
    parser.add_argument("--x", type=float, default=0.9)
    parser.add_argument("--y", type=float, default=0.5)
    parser.add_argument("--z", type=float, default=0.9)
    parser.add_argument("--roll", type=float, default=None)
    parser.add_argument("--pitch", type=float, default=None)
    parser.add_argument("--yaw", type=float, default=None)
    parser.add_argument("--arduino-port", default="/dev/ttyACM0")
    parser.add_argument("--arduino-baud", type=int, default=9600)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv if argv is not None else os.sys.argv[1:])

    rclpy.init()
    node = HomePlusIKPipeline()
    node.configure_arduino(args.arduino_port, args.arduino_baud)

    try:
        if args.roll is None or args.pitch is None or args.yaw is None:
            node.set_target_point(args.x, args.y, args.z)
        else:
            node.set_target_point_rpy(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino_serial and node.arduino_serial.is_open:
            node.arduino_serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
