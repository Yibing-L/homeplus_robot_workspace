#!/usr/bin/env python3

import math
import time
import threading
from copy import deepcopy
from pathlib import Path
from typing import Dict, List, Optional

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float32, String

# Joint order that the Arduino expects for trajectory waypoints
TRAJECTORY_JOINT_ORDER = [
    "base_x", "base_y", "base_theta",
    "joint_grip_L", "joint_hand", "joint_wrist", "joint_elbow", "joint_shoulder",
]


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
        self.declare_parameter("busy_topic", "/gesture_control/executor_busy")
        self.declare_parameter("target_pose_topic", "/gesture_control/target_pose")
        self.declare_parameter("joint_target_topic", "/gesture_control/joint_target")
        self.declare_parameter("gripper_target_topic", "/gesture_control/gripper_target")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("enable_moveit_planning", False)
        self.declare_parameter("move_group_action_name", "/move_action")
        self.declare_parameter("default_group_name", "arm")
        self.declare_parameter("end_effector_link", "palm_1")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("planner_id", "BiTRRTkConfigDefault")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("position_tolerance_m", 0.05)
        self.declare_parameter("orientation_tolerance_rad", 0.25)
        self.declare_parameter("joint_tolerance", 0.02)
        self.declare_parameter("workspace_bounds", [-2.0, -2.0, -1.0, 2.0, 2.0, 2.0])
        self.declare_parameter("gripper_settle_sec", 1.0)

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
        self.gripper_settle_sec = float(self.get_parameter("gripper_settle_sec").value)

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.busy_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("busy_topic").value),
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
        self.arduino_cmd_pub = self.create_publisher(
            String, "/arduino/command_queue", 10
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )

        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            GestureCommand,
            str(self.get_parameter("command_topic").value),
            self.command_callback,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.joint_states_callback,
            10,
            callback_group=self.cb_group,
        )

        self.latest_joint_state: Optional[JointState] = None
        self._busy = False
        self._busy_lock = threading.Lock()

        # Event used to signal when a MoveIt plan+execute completes
        self._plan_done_event = threading.Event()
        self._plan_success = False

        # Detection pose tracking — subscriptions created dynamically per sequence
        self._detection_pose: Optional[PoseStamped] = None
        self._detection_lock = threading.Lock()
        self._detection_subs: Dict[str, object] = {}  # topic -> subscription

        # Arduino execution tracking — wait for "DONE" ack
        self._arduino_done_event = threading.Event()
        self.create_subscription(
            String,
            "/arduino/ack",
            self._arduino_ack_callback,
            10,
            callback_group=self.cb_group,
        )

        self.move_group_client = None
        if self.enable_moveit_planning:
            self.move_group_client = ActionClient(
                self,
                MoveGroup,
                str(self.get_parameter("move_group_action_name").value),
                callback_group=self.cb_group,
            )
            self.get_logger().info("MoveIt planning is enabled for gesture execution.")

        self.get_logger().info(
            f"Gesture motion executor ready with {len(self.commands)} commands from {execution_map_path}"
        )

    # ------------------------------------------------------------------
    # Busy state
    # ------------------------------------------------------------------
    def _set_busy(self, busy: bool) -> None:
        with self._busy_lock:
            self._busy = busy
        self.busy_pub.publish(Bool(data=busy))

    def _is_busy(self) -> bool:
        with self._busy_lock:
            return self._busy

    # ------------------------------------------------------------------
    # Detection pose tracking
    # ------------------------------------------------------------------
    def _ensure_detection_subscription(self, topic: str) -> None:
        if topic in self._detection_subs:
            return
        self.get_logger().info(f"Subscribing to detection topic: {topic}")
        sub = self.create_subscription(
            PoseStamped,
            topic,
            self._detection_pose_callback,
            10,
            callback_group=self.cb_group,
        )
        self._detection_subs[topic] = sub

    def _detection_pose_callback(self, msg: PoseStamped) -> None:
        with self._detection_lock:
            self._detection_pose = msg

    def _get_detection_pose(self) -> Optional[PoseStamped]:
        with self._detection_lock:
            return deepcopy(self._detection_pose) if self._detection_pose else None

    def _wait_for_detection(self, timeout_sec: float) -> Optional[PoseStamped]:
        """Poll for a detection pose, returning None on timeout."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            pose = self._get_detection_pose()
            if pose is not None:
                return pose
            time.sleep(0.1)
        return None

    def _resolve_detected_pose(self, step: Dict, detected: PoseStamped) -> Dict:
        """Return a copy of the step config with position filled from detection + offsets."""
        resolved = dict(step)
        ox = float(step.get("offset_x", 0.0))
        oy = float(step.get("offset_y", 0.0))
        oz = float(step.get("offset_z", 0.0))
        resolved["position"] = [
            detected.pose.position.x + ox,
            detected.pose.position.y + oy,
            detected.pose.position.z + oz,
        ]
        return resolved

    # ------------------------------------------------------------------
    # Arduino execution tracking
    # ------------------------------------------------------------------
    def _arduino_ack_callback(self, msg: String) -> None:
        if msg.data.strip().upper() == "DONE":
            self._arduino_done_event.set()

    def _wait_for_arduino_done(self, timeout_sec: float) -> bool:
        """Block until Arduino sends DONE, or timeout."""
        self._arduino_done_event.clear()
        success = self._arduino_done_event.wait(timeout=timeout_sec)
        if not success:
            self.get_logger().warn(f"Arduino did not send DONE within {timeout_sec}s")
        return success

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------
    def _load_execution_map(self, path: str) -> Dict[str, Dict]:
        with open(path, "r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        commands = payload.get("commands", {})
        if not commands:
            raise ValueError(f"No commands found in {path}")
        return commands

    def joint_states_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------
    def command_callback(self, msg: GestureCommand) -> None:
        if self._is_busy():
            self._publish_status(
                f"Busy — ignoring gesture {msg.gesture_id} ({msg.command_name})"
            )
            self.get_logger().warn(
                f"Ignoring command {msg.command_name}: executor is busy"
            )
            return

        command_name = str(msg.command_name)
        command_cfg = self.commands.get(command_name)
        if command_cfg is None:
            self._publish_status(f"No execution configured for command {command_name}")
            self.get_logger().warn(f"No execution configured for command {command_name}")
            return

        action_type = str(command_cfg.get("type", msg.command_type or "command"))

        if action_type == "sequence":
            # Run the sequence on a background thread so we don't block
            # the executor's callback group while waiting for each step.
            thread = threading.Thread(
                target=self._run_sequence,
                args=(command_name, command_cfg),
                daemon=True,
            )
            thread.start()
        else:
            self._execute_single_step(command_name, command_cfg, msg.command_type)

    # ------------------------------------------------------------------
    # Single-step execution (unchanged behavior for non-sequence commands)
    # ------------------------------------------------------------------
    def _execute_single_step(
        self, command_name: str, command_cfg: Dict, fallback_type: str = ""
    ) -> None:
        action_type = str(command_cfg.get("type", fallback_type or "command"))
        self._publish_status(f"Executing {command_name} (type={action_type})")

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
            self.get_logger().warn(
                f"Unsupported execution type '{action_type}' for {command_name}"
            )

    # ------------------------------------------------------------------
    # Sequence execution
    # ------------------------------------------------------------------
    def _run_sequence(self, sequence_name: str, sequence_cfg: Dict) -> None:
        steps: List[Dict] = sequence_cfg.get("steps", [])
        if not steps:
            self.get_logger().error(f"Sequence {sequence_name} has no steps")
            return

        self._set_busy(True)
        self._publish_status(f"Starting sequence: {sequence_name} ({len(steps)} steps)")
        self.get_logger().info(
            f"=== Sequence '{sequence_name}' started ({len(steps)} steps) ==="
        )

        # If the sequence uses detection, subscribe and wait for a pose
        detection_topic = sequence_cfg.get("detection_topic")
        detected_pose: Optional[PoseStamped] = None
        needs_detection = detection_topic and any(
            s.get("target") == "detected" for s in steps
        )

        if needs_detection:
            self._ensure_detection_subscription(detection_topic)
            # Clear stale detection so we get a fresh one
            with self._detection_lock:
                self._detection_pose = None

            timeout = float(sequence_cfg.get("detection_timeout_sec", 5.0))
            self._publish_status(
                f"[{sequence_name}] Waiting for object detection on {detection_topic}..."
            )
            detected_pose = self._wait_for_detection(timeout)
            if detected_pose is None:
                self._abort_sequence(
                    sequence_name, "detection", f"no detection on {detection_topic} within {timeout}s"
                )
                return
            p = detected_pose.pose.position
            self.get_logger().info(
                f"  Detection acquired: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
            )

        for i, step in enumerate(steps):
            step_name = step.get("name", f"step_{i}")
            step_type = step.get("type", "command")

            # Resolve detected positions
            effective_step = step
            if step.get("target") == "detected" and detected_pose is not None:
                effective_step = self._resolve_detected_pose(step, detected_pose)
                pos = effective_step["position"]
                self.get_logger().info(
                    f"  {step_name}: resolved target → [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
                )

            self._publish_status(
                f"[{sequence_name}] step {i+1}/{len(steps)}: {step_name}"
            )
            self.get_logger().info(
                f"  Step {i+1}/{len(steps)}: {step_name} (type={step_type})"
            )

            if step_type == "gripper":
                grip_val = float(effective_step["value"])
                self.gripper_target_pub.publish(Float32(data=grip_val))
                self._send_gripper_to_arduino(grip_val)
                # Wait for Arduino to finish gripper movement
                self.get_logger().info(f"  Waiting for Arduino to finish gripper...")
                self._wait_for_arduino_done(timeout_sec=15.0)

            elif step_type == "pose":
                if not self.enable_moveit_planning or self.move_group_client is None:
                    self._abort_sequence(sequence_name, step_name, "MoveIt not enabled")
                    return

                pose_msg = self._build_pose_message(effective_step)
                self.target_pose_pub.publish(pose_msg)

                self._plan_done_event.clear()
                self._plan_success = False
                self._plan_pose(step_name, pose_msg, effective_step)

                self._plan_done_event.wait(timeout=30.0)
                if not self._plan_success:
                    self._abort_sequence(sequence_name, step_name, "planning/execution failed")
                    return

                # Wait for Arduino to finish executing trajectory
                self.get_logger().info(f"  Waiting for Arduino to finish trajectory...")
                self._wait_for_arduino_done(timeout_sec=60.0)

            elif step_type == "joint_state":
                if not self.enable_moveit_planning or self.move_group_client is None:
                    self._abort_sequence(sequence_name, step_name, "MoveIt not enabled")
                    return

                joint_msg = self._build_joint_state_message(effective_step)
                self.joint_target_pub.publish(joint_msg)

                self._plan_done_event.clear()
                self._plan_success = False
                self._plan_joint_state(step_name, joint_msg, effective_step)

                self._plan_done_event.wait(timeout=30.0)
                if not self._plan_success:
                    self._abort_sequence(sequence_name, step_name, "planning/execution failed")
                    return

                # Wait for Arduino to finish executing trajectory
                self.get_logger().info(f"  Waiting for Arduino to finish trajectory...")
                self._wait_for_arduino_done(timeout_sec=60.0)
            else:
                self.get_logger().warn(f"Unknown step type '{step_type}' in {step_name}")

        self._set_busy(False)
        self._publish_status(f"Sequence {sequence_name} completed successfully")
        self.get_logger().info(f"=== Sequence '{sequence_name}' completed ===")

    def _abort_sequence(self, sequence_name: str, step_name: str, reason: str) -> None:
        self._set_busy(False)
        self._publish_status(
            f"Sequence {sequence_name} ABORTED at {step_name}: {reason}"
        )
        self.get_logger().error(
            f"=== Sequence '{sequence_name}' ABORTED at {step_name}: {reason} ==="
        )

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Trajectory extraction and Arduino formatting
    # ------------------------------------------------------------------
    def _extract_and_send_trajectory(self, name: str, planned_trajectory) -> None:
        """Extract waypoints from a MoveIt planned trajectory and send to Arduino."""
        jt = planned_trajectory.joint_trajectory
        if not jt.points:
            self.get_logger().warn(f"No waypoints in trajectory for {name}")
            return

        joint_indices = {jn: idx for idx, jn in enumerate(jt.joint_names)}

        # Log raw MoveIt joint names and trajectory info
        self.get_logger().info(f"  MoveIt joints: {jt.joint_names}")
        self.get_logger().info(f"  Trajectory has {len(jt.points)} waypoints")

        arduino_waypoints = []
        raw_waypoints = []
        for point in jt.points:
            values = []
            for joint_name in TRAJECTORY_JOINT_ORDER:
                idx = joint_indices.get(joint_name)
                values.append(point.positions[idx] if idx is not None else 0.0)
            raw_waypoints.append(list(values))
            arduino_waypoints.append(self._format_waypoint_for_arduino(values))

        # Log first and last waypoint (raw radians/meters and Arduino format)
        self.get_logger().info(f"  Raw first: {raw_waypoints[0]}")
        self.get_logger().info(f"  Raw last:  {raw_waypoints[-1]}")
        self.get_logger().info(f"  Arduino first: {arduino_waypoints[0]}")
        self.get_logger().info(f"  Arduino last:  {arduino_waypoints[-1]}")

        # Save trajectory to CSV
        self._save_trajectory_csv(name, jt, raw_waypoints, arduino_waypoints)

        # Send as comma-separated queue
        command_queue = ",".join(arduino_waypoints)
        self.arduino_cmd_pub.publish(String(data=command_queue))
        self.get_logger().info(
            f"Sent {len(arduino_waypoints)} waypoints to Arduino for {name}"
        )

        # Update joint state to the final waypoint so the next step plans from here
        self._publish_joint_state(raw_waypoints[-1])

    def _save_trajectory_csv(self, name: str, jt, raw_waypoints, arduino_waypoints) -> None:
        """Save planned trajectory to CSV for debugging."""
        import csv
        from datetime import datetime
        from pathlib import Path

        log_dir = Path("/mnt/c/users/easha/arl/homeplus_robot_workspace/trajectory_logs")
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = log_dir / f"traj_{name}_{timestamp}.csv"

        try:
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                # Header
                raw_header = [f"raw_{j}" for j in TRAJECTORY_JOINT_ORDER]
                arduino_header = ["arduino_x_cm", "arduino_y_cm", "arduino_theta_deg",
                                  "arduino_grip", "arduino_hand_deg", "arduino_wrist_deg",
                                  "arduino_elbow_deg", "arduino_shoulder_deg",
                                  "arduino_la", "arduino_frame"]
                writer.writerow(["waypoint", "time_from_start"] + raw_header + arduino_header)

                for i, point in enumerate(jt.points):
                    t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                    arduino_vals = arduino_waypoints[i].split()
                    writer.writerow([i, f"{t:.3f}"] + [f"{v:.6f}" for v in raw_waypoints[i]] + arduino_vals)

            self.get_logger().info(f"  Trajectory saved: {csv_path}")
        except Exception as e:
            self.get_logger().warn(f"  Failed to save trajectory CSV: {e}")

    @staticmethod
    def _format_waypoint_for_arduino(values: List[float]) -> str:
        """Convert joint values to Arduino format: m→cm, rad→deg."""
        if len(values) < 8:
            return "0 0 0 0 0 0 0 0"

        base_x_cm = values[0] * 100.0         # m → cm
        base_y_cm = values[1] * 100.0         # m → cm
        theta_deg = math.degrees(values[2])   # rad → deg
        grip = values[3]                       # pass through (0-90 degrees)
        hand_deg = math.degrees(values[4])    # rad → deg
        wrist_deg = math.degrees(values[5])   # rad → deg
        elbow_deg = math.degrees(values[6])   # rad → deg
        shoulder_deg = math.degrees(values[7])  # rad → deg
        la = 0.0                               # linear actuator (unused)
        frame = 0.0                            # frame (unused)

        return (
            f"{base_x_cm:.1f} {base_y_cm:.1f} {theta_deg:.1f} "
            f"{grip:.1f} {hand_deg:.1f} {wrist_deg:.1f} "
            f"{elbow_deg:.1f} {shoulder_deg:.1f} {la:.1f} {frame:.1f}"
        )

    def _publish_joint_state(self, positions: List[float]) -> None:
        """Publish joint positions so MoveIt knows where the arm is after a trajectory."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(TRAJECTORY_JOINT_ORDER)
        js.position = list(positions)
        self.joint_state_pub.publish(js)
        # Also update our own cached state
        self.latest_joint_state = js

    def _send_gripper_to_arduino(self, grip_value: float) -> None:
        """Send a full 10-value waypoint to Arduino with current joint positions and updated grip."""
        if self.latest_joint_state is None:
            self.get_logger().warn("No joint state available — cannot send gripper command")
            return

        current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        values = []
        for joint_name in TRAJECTORY_JOINT_ORDER:
            if joint_name == "joint_grip_L":
                values.append(grip_value)
            else:
                values.append(current.get(joint_name, 0.0))

        waypoint = self._format_waypoint_for_arduino(values)
        self.arduino_cmd_pub.publish(String(data=waypoint))
        self.get_logger().info(f"Sent gripper waypoint to Arduino: {waypoint}")

        # Update joint state with the new grip value
        self._publish_joint_state(values)

    # ------------------------------------------------------------------
    # Gripper value note: 0 = closed, 90 = fully open
    # Arduino expects 10 space-separated values per waypoint:
    #   x_cm y_cm theta_deg grip hand wrist elbow shoulder la frame
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # MoveIt planning
    # ------------------------------------------------------------------
    def _plan_pose(self, name: str, pose_msg: PoseStamped, command_cfg: Dict) -> None:
        if self.move_group_client is None:
            return
        if not self.move_group_client.server_is_ready():
            self.get_logger().warn("MoveIt action server is not ready.")
            self._signal_plan_done(False)
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
        if not self.move_group_client.server_is_ready():
            self.get_logger().warn("MoveIt action server is not ready.")
            self._signal_plan_done(False)
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

    # ------------------------------------------------------------------
    # MoveIt action handling
    # ------------------------------------------------------------------
    def _send_move_group_goal(self, name: str, goal: MoveGroup.Goal) -> None:
        self._publish_status(f"Sending MoveIt request for {name}")
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._goal_response_callback(name, done))

    def _goal_response_callback(self, name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"MoveIt request for {name} failed before acceptance: {exc}")
            self._publish_status(f"MoveIt request for {name} failed: {exc}")
            self._signal_plan_done(False)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"MoveIt request for {name} was rejected")
            self._publish_status(f"MoveIt request for {name} was rejected")
            self._signal_plan_done(False)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._goal_result_callback(name, done))

    def _goal_result_callback(self, name: str, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"MoveIt result retrieval failed for {name}: {exc}")
            self._publish_status(f"MoveIt result retrieval failed for {name}: {exc}")
            self._signal_plan_done(False)
            return

        error_code = int(result.error_code.val)
        if error_code == 1:
            self.get_logger().info(f"MoveIt planning succeeded for {name}")
            self._publish_status(f"MoveIt planning succeeded for {name}")

            # Extract trajectory and send to Arduino
            trajectory = result.planned_trajectory
            if trajectory and trajectory.joint_trajectory.points:
                self._extract_and_send_trajectory(name, trajectory)
            else:
                self.get_logger().warn(f"No trajectory in result for {name}")

            self._signal_plan_done(True)
        else:
            self.get_logger().warn(f"MoveIt failed for {name} with code {error_code}")
            self._publish_status(f"MoveIt failed for {name} with code {error_code}")
            self._signal_plan_done(False)

    def _signal_plan_done(self, success: bool) -> None:
        self._plan_success = success
        self._plan_done_event.set()

    def _publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureMotionExecutor()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
