#!/usr/bin/env python3
"""Replay a saved trajectory CSV in RViz via /display_planned_path."""

import csv
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

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


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 replay_trajectory.py <trajectory_csv>")
        print("Example: python3 replay_trajectory.py trajectory_logs/traj_move_to_target_*.csv")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not Path(csv_path).exists():
        print(f"File not found: {csv_path}")
        sys.exit(1)

    waypoints = load_csv(csv_path)
    print(f"Loaded {len(waypoints)} waypoints from {csv_path}")
    print(f"First: t={waypoints[0][0]:.3f}s pos={[f'{v:.3f}' for v in waypoints[0][1]]}")
    print(f"Last:  t={waypoints[-1][0]:.3f}s pos={[f'{v:.3f}' for v in waypoints[-1][1]]}")

    rclpy.init()
    node = Node("trajectory_replayer")
    pub = node.create_publisher(DisplayTrajectory, "/display_planned_path", 10)

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

    rt = RobotTrajectory()
    rt.joint_trajectory = jt

    msg = DisplayTrajectory()
    msg.model_id = "homeplus_urdf"
    msg.trajectory = [rt]

    # Publish a few times to make sure RViz picks it up
    import time
    print("Publishing trajectory to /display_planned_path ...")
    for i in range(10):
        pub.publish(msg)
        time.sleep(0.5)
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Done. Check RViz — the trajectory should appear in the MotionPlanning display.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
