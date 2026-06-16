#!/usr/bin/env python3
"""Replay a saved trajectory CSV in RViz via /display_planned_path."""

import argparse
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker

JOINT_ORDER = [
    "base_x", "base_y", "base_theta",
    "joint_grip_L", "joint_hand", "joint_wrist", "joint_elbow", "joint_shoulder",
]


def load_csv(path: str):
    waypoints = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["time_from_start"])
            positions = []
            for joint in JOINT_ORDER:
                positions.append(float(row[f"raw_{joint}"]))
            waypoints.append((t, positions))
    return waypoints


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay a saved HomePlus trajectory CSV in RViz."
    )
    parser.add_argument("trajectory_csv")
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=30.0,
        help="Seconds to keep republishing after initial display. Use 0 to exit immediately.",
    )
    parser.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Publish a target marker at this point.",
    )
    parser.add_argument(
        "--target-frame",
        default="world",
        help="Frame for --target marker.",
    )
    parser.add_argument(
        "--target-from-topic",
        action="store_true",
        help="Use the latest /gdino_pose_world pose as the target marker.",
    )
    parser.add_argument(
        "--target-topic",
        default="/gdino_pose_world",
        help="Pose topic to read when --target-from-topic is set.",
    )
    parser.add_argument(
        "--publish-final-state",
        action="store_true",
        help="Publish the final trajectory waypoint on /joint_states during hold. "
             "Leave off if another node is already publishing /joint_states.",
    )
    return parser.parse_args()


def make_target_markers(frame_id: str, x: float, y: float, z: float):
    sphere = Marker()
    sphere.header.frame_id = frame_id
    sphere.ns = "trajectory_replay"
    sphere.id = 1
    sphere.type = Marker.SPHERE
    sphere.action = Marker.ADD
    sphere.pose.position.x = float(x)
    sphere.pose.position.y = float(y)
    sphere.pose.position.z = float(z)
    sphere.pose.orientation.w = 1.0
    sphere.scale.x = 0.30
    sphere.scale.y = 0.30
    sphere.scale.z = 0.30
    sphere.color.r = 1.0
    sphere.color.g = 0.0
    sphere.color.b = 0.0
    sphere.color.a = 1.0

    text = Marker()
    text.header.frame_id = frame_id
    text.ns = "trajectory_replay"
    text.id = 2
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = float(x)
    text.pose.position.y = float(y)
    text.pose.position.z = float(z) + 0.35
    text.pose.orientation.w = 1.0
    text.scale.z = 0.18
    text.color.r = 1.0
    text.color.g = 1.0
    text.color.b = 0.0
    text.color.a = 1.0
    text.text = "DINO TARGET"

    return [sphere, text]


def publish_target_markers(marker_pub, markers, node):
    if not markers:
        return
    stamp = node.get_clock().now().to_msg()
    for marker in markers:
        marker.header.stamp = stamp
        marker_pub.publish(marker)


def make_target_point(frame_id: str, x: float, y: float, z: float) -> PointStamped:
    point = PointStamped()
    point.header.frame_id = frame_id
    point.point.x = float(x)
    point.point.y = float(y)
    point.point.z = float(z)
    return point


def publish_target_point(point_pub, point, node):
    if point is None:
        return
    point.header.stamp = node.get_clock().now().to_msg()
    point_pub.publish(point)


def make_final_joint_state(positions):
    msg = JointState()
    msg.name = list(JOINT_ORDER)
    msg.position = list(positions)
    return msg


def main():
    args = parse_args()

    csv_path = args.trajectory_csv
    if not Path(csv_path).exists():
        print(f"File not found: {csv_path}")
        raise SystemExit(1)

    waypoints = load_csv(csv_path)
    print(f"Loaded {len(waypoints)} waypoints from {csv_path}")
    print(f"First: t={waypoints[0][0]:.3f}s pos={[f'{v:.3f}' for v in waypoints[0][1]]}")
    print(f"Last:  t={waypoints[-1][0]:.3f}s pos={[f'{v:.3f}' for v in waypoints[-1][1]]}")

    rclpy.init()
    node = Node("trajectory_replayer")
    pub = node.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
    marker_pub = node.create_publisher(Marker, "/trajectory_replay/target_marker", 10)
    point_pub = node.create_publisher(PointStamped, "/trajectory_replay/target_point", 10)
    joint_pub = node.create_publisher(JointState, "/joint_states", 10)
    latest_target = {"pose": None}

    if args.target_from_topic:
        def target_callback(msg):
            latest_target["pose"] = msg

        node.create_subscription(PoseStamped, args.target_topic, target_callback, 10)
        print(f"Waiting up to 2s for target pose on {args.target_topic} ...")
        import time
        deadline = time.monotonic() + 2.0
        while latest_target["pose"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    # Build the trajectory message
    jt = JointTrajectory()
    jt.joint_names = list(JOINT_ORDER)

    for t, positions in waypoints:
        point = JointTrajectoryPoint()
        point.positions = positions
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        jt.points.append(point)

    if args.hold_sec > 0:
        hold_t = waypoints[-1][0] + args.hold_sec
        hold_point = JointTrajectoryPoint()
        hold_point.positions = list(waypoints[-1][1])
        sec = int(hold_t)
        nanosec = int((hold_t - sec) * 1e9)
        hold_point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        jt.points.append(hold_point)

    rt = RobotTrajectory()
    rt.joint_trajectory = jt

    msg = DisplayTrajectory()
    msg.model_id = "homeplus_urdf"
    msg.trajectory_start.joint_state.name = list(JOINT_ORDER)
    msg.trajectory_start.joint_state.position = list(waypoints[0][1])
    msg.trajectory = [rt]

    target_markers = []
    target_point = None
    if args.target is not None:
        target_markers = make_target_markers(args.target_frame, *args.target)
        target_point = make_target_point(args.target_frame, *args.target)
    elif latest_target["pose"] is not None:
        pose = latest_target["pose"]
        p = pose.pose.position
        target_markers = make_target_markers(args.target_frame, p.x, p.y, p.z)
        target_point = make_target_point(args.target_frame, p.x, p.y, p.z)
        print(
            f"Target marker from {args.target_topic}: "
            f"source_frame={pose.header.frame_id} marker_frame={args.target_frame} "
            f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
    elif args.target_from_topic:
        print(f"No target pose received on {args.target_topic}; replaying trajectory only.")

    final_joint_state = make_final_joint_state(waypoints[-1][1])

    # Publish a few times to make sure RViz picks it up
    import time
    print("Publishing trajectory to /display_planned_path ...")
    for i in range(10):
        pub.publish(msg)
        publish_target_markers(marker_pub, target_markers, node)
        publish_target_point(point_pub, target_point, node)
        time.sleep(0.5)
        rclpy.spin_once(node, timeout_sec=0.1)

    if args.hold_sec > 0:
        print(
            f"Trajectory includes a {args.hold_sec:.1f}s final-pose hold. "
            f"Keeping target point/marker alive for "
            f"{args.hold_sec:.1f}s. Ctrl-C to stop early."
        )
        deadline = time.monotonic() + args.hold_sec
        while time.monotonic() < deadline and rclpy.ok():
            if args.publish_final_state:
                final_joint_state.header.stamp = node.get_clock().now().to_msg()
                joint_pub.publish(final_joint_state)
            publish_target_markers(marker_pub, target_markers, node)
            publish_target_point(point_pub, target_point, node)
            rclpy.spin_once(node, timeout_sec=0.1)

    print("Done. Check RViz — the trajectory should appear in the MotionPlanning display.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
