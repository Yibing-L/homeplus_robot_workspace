#!/usr/bin/env python3

import math
import json
import random
from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace
import time
import threading
import tempfile
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.serialization import serialize_message
import tf2_geometry_msgs
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from homeplus_interfaces.msg import GestureCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    CollisionObject, AttachedCollisionObject, AllowedCollisionEntry, PlanningSceneComponents,
    DisplayTrajectory, DisplayRobotState, ObjectColor,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetPositionFK, GetStateValidity, GetCartesianPath, ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float32, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

from planar_reach import PlanarArm, arm_plane_error, fixed_link_height, plan_base_placements, wrap

# Joint order that the Arduino expects for trajectory waypoints
TRAJECTORY_JOINT_ORDER = [
    "base_x", "base_y", "base_theta",
    "joint_grip_L", "joint_hand", "joint_wrist", "joint_elbow", "joint_shoulder",
]

WHEEL_JOINT_NAMES = [
    "joint_wheel_FL", "joint_wheel_FR", "joint_wheel_RL", "joint_wheel_RR",
]

BASE_JOINT_NAMES = ["base_x", "base_y", "base_theta"]
ARM_JOINT_NAMES = [
    "joint_shoulder", "joint_elbow", "joint_wrist", "joint_hand"
]

MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_ACQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}


class _DryRunPublisher:
    """Stands in for a hardware publisher when dry_run is set: logs, never sends."""

    def __init__(self, node, topic):
        self._node = node
        self._topic = topic

    def publish(self, msg):
        text = str(getattr(msg, 'data', msg))
        self._node.get_logger().info(
            f'DRY RUN: not publishing on {self._topic}: {text[:160]}'
            + ('...' if len(text) > 160 else ''))


@dataclass
class PlanContext:
    """State for one MoveGroup request.

    Keeping this state request-local prevents a late result from a timed-out
    plan from completing (or executing) a newer command.
    """

    request_id: int
    name: str
    event: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    cancelled: bool = False
    awaiting_approval: bool = False
    retryable_failure: bool = False
    goal_handle: object = None


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
    def __init__(self, **node_options):
        super().__init__("gesture_motion_executor", **node_options)

        self.declare_parameter("execution_map_path", "")
        self.declare_parameter("command_topic", "/gesture_control/command")
        self.declare_parameter("status_topic", "/gesture_control/executor_status")
        self.declare_parameter("busy_topic", "/gesture_control/executor_busy")
        self.declare_parameter("target_pose_topic", "/gesture_control/target_pose")
        self.declare_parameter("joint_target_topic", "/gesture_control/joint_target")
        self.declare_parameter("gripper_target_topic", "/gesture_control/gripper_target")
        self.declare_parameter("joint_states_topic", "/hardware/joint_states")
        self.declare_parameter("enable_moveit_planning", False)
        self.declare_parameter("move_group_action_name", "/move_action")
        self.declare_parameter("default_group_name", "base_arm")
        self.declare_parameter("execution_order", "staged")
        self.declare_parameter("planning_service_name", "/plan_kinematic_path")
        self.declare_parameter("end_effector_link", "grasp_tcp")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("wait_for_gripper_ack", True)
        self.declare_parameter("wait_for_trajectory_ack", True)
        self.declare_parameter("stream_trajectory_waypoints", False)
        self.declare_parameter("pretty_print_trajectory_waypoints", True)
        self.declare_parameter("arduino_wait_progress_sec", 2.0)
        self.declare_parameter("display_planned_path_topic", "/display_planned_path")
        self.declare_parameter("require_trajectory_approval", True)
        self.declare_parameter("trajectory_approval_topic", "/gesture_control/approve_planned_path")
        self.declare_parameter("trajectory_approval_timeout_sec", 60.0)
        self.declare_parameter("tf_lookup_timeout_sec", 0.5)
        self.declare_parameter("position_tolerance_m", 0.05)
        self.declare_parameter("use_orientation_constraint", False)
        self.declare_parameter("orientation_tolerance_rad", 0.25)
        self.declare_parameter("joint_tolerance", 0.02)
        self.declare_parameter("workspace_bounds", [-2.0, -2.0, -1.0, 2.0, 2.0, 2.0])
        self.declare_parameter("gripper_settle_sec", 1.0)
        self.declare_parameter("continue_on_arduino_timeout", False)
        self.declare_parameter("max_start_state_error", 0.03)
        self.declare_parameter("joint_state_max_age_sec", 0.5)
        self.declare_parameter("detection_max_age_sec", 15.0)
        self.declare_parameter("detection_min_stable_samples", 1)
        self.declare_parameter("detection_stability_tolerance_m", 0.05)
        self.declare_parameter("detection_max_refresh_shift_m", 0.0)
        self.declare_parameter("clear_octomap_before_pose_plan", True)
        self.declare_parameter("clear_octomap_service", "/clear_octomap")
        self.declare_parameter("octomap_clear_timeout_sec", 2.0)
        self.declare_parameter("octomap_rebuild_wait_sec", 1.2)
        # Plan and validate every step, but never publish to the Arduino:
        # motions complete instantly on a virtual state overlaid on the
        # measured feedback. Use a separate command topic from the live node.
        self.declare_parameter("dry_run", False)
        self.declare_parameter("robot_description_topic", "/robot_description")

        execution_map_path = str(self.get_parameter("execution_map_path").value)
        if not execution_map_path:
            raise ValueError("Parameter 'execution_map_path' is required.")
        if not Path(execution_map_path).is_file():
            raise FileNotFoundError(f"Execution map not found: {execution_map_path}")

        self.commands = self._load_execution_map(execution_map_path)
        self.enable_moveit_planning = bool(self.get_parameter("enable_moveit_planning").value)
        self.default_group_name = str(self.get_parameter("default_group_name").value)
        self.execution_order = str(
            self.get_parameter("execution_order").value
        ).strip().lower()
        if self.execution_order not in {"coordinated", "staged"}:
            raise ValueError(
                "execution_order must be 'staged' or 'coordinated'; legacy path replacement is disabled"
            )
        self.end_effector_link = str(self.get_parameter("end_effector_link").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.wait_for_gripper_ack = bool(self.get_parameter("wait_for_gripper_ack").value)
        self.wait_for_trajectory_ack = bool(self.get_parameter("wait_for_trajectory_ack").value)
        self.stream_trajectory_waypoints = bool(self.get_parameter("stream_trajectory_waypoints").value)
        self.pretty_print_trajectory_waypoints = bool(
            self.get_parameter("pretty_print_trajectory_waypoints").value
        )
        self.arduino_wait_progress_sec = float(
            self.get_parameter("arduino_wait_progress_sec").value
        )
        self.require_trajectory_approval = bool(
            self.get_parameter("require_trajectory_approval").value
        )
        self.trajectory_approval_topic = str(
            self.get_parameter("trajectory_approval_topic").value
        )
        self.trajectory_approval_timeout_sec = float(
            self.get_parameter("trajectory_approval_timeout_sec").value
        )
        self.tf_lookup_timeout_sec = float(self.get_parameter("tf_lookup_timeout_sec").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.use_orientation_constraint = bool(
            self.get_parameter("use_orientation_constraint").value
        )
        self.orientation_tolerance_rad = float(self.get_parameter("orientation_tolerance_rad").value)
        self.joint_tolerance = float(self.get_parameter("joint_tolerance").value)
        self.workspace_bounds = list(self.get_parameter("workspace_bounds").value)
        self.gripper_settle_sec = float(self.get_parameter("gripper_settle_sec").value)
        self.continue_on_arduino_timeout = bool(
            self.get_parameter("continue_on_arduino_timeout").value
        )
        self.detection_min_stable_samples = max(
            1, int(self.get_parameter("detection_min_stable_samples").value)
        )
        self.detection_stability_tolerance_m = float(
            self.get_parameter("detection_stability_tolerance_m").value
        )
        self.detection_max_refresh_shift_m = float(
            self.get_parameter("detection_max_refresh_shift_m").value
        )
        self.max_start_state_error = float(
            self.get_parameter("max_start_state_error").value
        )
        self.clear_octomap_before_pose_plan = bool(
            self.get_parameter("clear_octomap_before_pose_plan").value
        )
        self.octomap_clear_timeout_sec = max(
            0.1, float(self.get_parameter("octomap_clear_timeout_sec").value)
        )
        self.octomap_rebuild_wait_sec = max(
            0.0, float(self.get_parameter("octomap_rebuild_wait_sec").value)
        )

        if not self.wait_for_trajectory_ack or not self.wait_for_gripper_ack:
            raise ValueError('Hardware executor requires measured acknowledgements; use RViz display for offline plans')
        if self.continue_on_arduino_timeout:
            raise ValueError('continue_on_arduino_timeout is unsupported: failures must stop motion')
        if self.stream_trajectory_waypoints:
            raise ValueError('Protocol v2 requires an atomic timed queue')
        if not self.plan_only:
            raise ValueError(
                "plan_only must remain true: this node executes trajectories through "
                "the Arduino bridge, so MoveIt execution would command the motion twice."
            )

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
        self.stop_pub = self.create_publisher(Bool, "/arduino/stop", 10)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self._virtual_positions: Dict[str, float] = {}
        if self.dry_run:
            self.arduino_cmd_pub = _DryRunPublisher(self, "/arduino/command_queue")
            self.stop_pub = _DryRunPublisher(self, "/arduino/stop")
            self.get_logger().warn("DRY RUN: trajectories and gripper commands are never sent")
        self._hardware_ready = False
        self.create_subscription(Bool, "/arduino/ready", self._ready_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.display_trajectory_pub = self.create_publisher(
            DisplayTrajectory,
            str(self.get_parameter("display_planned_path_topic").value),
            10,
        )

        debug_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.collision_state_pub = self.create_publisher(
            DisplayRobotState, '/planning_debug/rejected_state', debug_qos)
        self.collision_markers_pub = self.create_publisher(
            MarkerArray, '/planning_debug/contacts', debug_qos)
        self._collision_debug = None
        # The MoveIt RobotState RViz plugin may use volatile durability. Repeat
        # the last diagnostic for late subscribers without touching robot state.
        self.create_timer(1.0, self._republish_collision_debug)

        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.octomap_clear_client = self.create_client(
            Empty,
            str(self.get_parameter("clear_octomap_service").value),
            callback_group=self.cb_group,
        )
        self.motion_plan_client = self.create_client(
            GetMotionPlan,
            str(self.get_parameter("planning_service_name").value),
            callback_group=self.cb_group,
        )
        self.detector_config_client = self.create_client(SetParameters, '/molmopoint/set_parameters', callback_group=self.cb_group)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.cb_group)
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=self.cb_group)
        self._cup_geometry = None
        self._cup_collision_object = None
        self._holding_cup = False
        self._base_link_height = 0.0
        # Arm reach model for deliberate base placement, built from the URDF.
        self._robot_description: Optional[str] = None
        self._planar_arms: Dict[str, PlanarArm] = {}
        self.create_subscription(
            String, str(self.get_parameter("robot_description_topic").value),
            self._robot_description_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.cb_group)
        self.create_subscription(String, '/molmopoint/geometry', self._geometry_callback, 10, callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk', callback_group=self.cb_group)
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity', callback_group=self.cb_group)
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb_group)
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
        self.create_subscription(
            Bool,
            self.trajectory_approval_topic,
            self._trajectory_approval_callback,
            10,
            callback_group=self.cb_group,
        )

        self.latest_joint_state: Optional[JointState] = None
        self._joint_history = deque(maxlen=40)
        self._last_state_received = 0.0
        self._command_id = 0
        self._command_started = 0.0
        self._cloud_stamp_ns = 0
        self.create_subscription(String, '/vision/planning_cloud_ready', self._cloud_ready_callback, 10,
                                 callback_group=self.cb_group)
        self._busy = False
        self._busy_lock = threading.Lock()

        self._plan_lock = threading.Lock()
        self._next_plan_request_id = 1
        self._active_plan: Optional[PlanContext] = None
        self._trajectory_approval_event = threading.Event()
        self._trajectory_approval_response: Optional[bool] = None
        self._trajectory_approval_lock = threading.Lock()

        # Detection pose tracking — subscriptions created dynamically per sequence
        self._detection_poses: Dict[str, PoseStamped] = {}
        self._detection_samples: Dict[str, List[PoseStamped]] = {}
        self._detection_lock = threading.Lock()
        self._detection_after_ns = 0
        self._detection_subs: Dict[str, object] = {}  # topic -> subscription

        # Arduino execution tracking — wait for "DONE" ack
        self._arduino_done_event = threading.Event()
        self._arduino_result_lock = threading.Lock()
        self._arduino_error_message: Optional[str] = None
        self._pending_joint_state: Optional[List[float]] = None
        self._pending_joint_state_reason: str = ""
        self._pending_joint_state_lock = threading.Lock()
        self.create_subscription(
            String,
            "/arduino/ack",
            self._arduino_ack_callback,
            10,
            callback_group=self.cb_group,
        )

        # Sequence-level trajectory logging. Individual MoveIt steps are still
        # saved separately; this buffer also saves the whole sequence as one CSV.
        self._sequence_log_name: Optional[str] = None
        self._sequence_log_rows: List[Dict] = []
        self._sequence_log_time_offset = 0.0
        self._target_audit_path: Optional[Path] = None

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
        self.get_logger().info(
            f"Hardware trajectory execution order: {self.execution_order}"
        )

    # ------------------------------------------------------------------
    # Busy state
    # ------------------------------------------------------------------
    def _set_busy(self, busy: bool) -> None:
        with self._busy_lock:
            self._busy = busy
        self.busy_pub.publish(Bool(data=busy))

    def _try_set_busy(self) -> bool:
        """Atomically claim the executor for one command."""
        with self._busy_lock:
            if self._busy:
                return False
            self._busy = True
        self.busy_pub.publish(Bool(data=True))
        return True

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
            lambda msg, source_topic=topic: self._detection_pose_callback(source_topic, msg),
            10,
            callback_group=self.cb_group,
        )
        self._detection_subs[topic] = sub

    def _detection_pose_callback(self, topic: str, msg: PoseStamped) -> None:
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= self._detection_after_ns or age < -0.1 or age > float(self.get_parameter('detection_max_age_sec').value):
            return
        if msg.header.frame_id != self.base_frame or not all(math.isfinite(v) for v in
                (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)):
            return
        with self._detection_lock:
            existing = self._detection_samples.get(topic, [])
            if existing and stamp <= Time.from_msg(existing[-1].header.stamp).nanoseconds:
                return
            self._detection_poses[topic] = msg
            samples = self._detection_samples.setdefault(topic, [])
            samples.append(deepcopy(msg))
            if len(samples) > 30:
                del samples[:-30]

    def _get_detection_pose(self, topic: str) -> Optional[PoseStamped]:
        with self._detection_lock:
            pose = self._detection_poses.get(topic)
            return deepcopy(pose) if pose else None

    def _wait_for_detection(
        self,
        topic: str,
        timeout_sec: float,
        reference_pose: Optional[PoseStamped] = None,
        required_samples: Optional[int] = None,
        stability_tolerance_m: Optional[float] = None,
        max_refresh_shift_m: Optional[float] = None,
    ) -> Optional[PoseStamped]:
        """Accept one fresh detection by default; multi-sample filtering is opt-in.

        With required > 1 the latest `required` samples must all lie within
        `stability_tolerance_m` of each other (pairwise). An outlier is never
        averaged in: it blocks acceptance until enough newer consistent samples
        push it out of the window, and the result is their per-axis median.
        """
        deadline = time.monotonic() + timeout_sec
        required = max(
            1,
            int(
                self.detection_min_stable_samples
                if required_samples is None
                else required_samples
            ),
        )
        tolerance = (
            self.detection_stability_tolerance_m
            if stability_tolerance_m is None
            else float(stability_tolerance_m)
        )
        max_refresh_shift = (
            self.detection_max_refresh_shift_m
            if max_refresh_shift_m is None
            else float(max_refresh_shift_m)
        )
        next_progress_log = time.monotonic() + 2.0
        received_count = 0
        usable_count = 0
        rejected_by_reference = 0
        latest_deviation: Optional[float] = None

        while time.monotonic() < deadline:
            with self._detection_lock:
                samples = [deepcopy(pose) for pose in self._detection_samples.get(topic, [])]

            received_count = len(samples)
            if reference_pose is not None and max_refresh_shift > 0:
                reference = reference_pose.pose.position
                accepted_samples = []
                for pose in samples:
                    shift = math.dist(
                        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                        (reference.x, reference.y, reference.z),
                    )
                    if shift <= max_refresh_shift:
                        accepted_samples.append(pose)
                samples = accepted_samples
            usable_count = len(samples)
            rejected_by_reference = received_count - usable_count

            if required == 1 and samples:
                candidate = samples[-1]
                age = (self.get_clock().now().nanoseconds -
                       Time.from_msg(candidate.header.stamp).nanoseconds) / 1e9
                if -0.1 <= age <= float(self.get_parameter('detection_max_age_sec').value):
                    self.get_logger().info('  Detection acquired from one fresh observation')
                    return deepcopy(candidate)

            if required > 1 and len(samples) >= required:
                candidates = samples[-required:]
                xs = sorted(pose.pose.position.x for pose in candidates)
                ys = sorted(pose.pose.position.y for pose in candidates)
                zs = sorted(pose.pose.position.z for pose in candidates)
                middle = required // 2
                if required % 2:
                    median_xyz = (xs[middle], ys[middle], zs[middle])
                else:
                    median_xyz = (
                        0.5 * (xs[middle - 1] + xs[middle]),
                        0.5 * (ys[middle - 1] + ys[middle]),
                        0.5 * (zs[middle - 1] + zs[middle]),
                    )

                max_deviation = max(
                    math.dist(
                        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                        median_xyz,
                    )
                    for pose in candidates
                )
                diameter = max(math.dist((a.pose.position.x, a.pose.position.y, a.pose.position.z),
                                         (b.pose.position.x, b.pose.position.y, b.pose.position.z))
                               for a in candidates for b in candidates)
                latest_deviation = diameter
                ages = [(self.get_clock().now().nanoseconds - Time.from_msg(p.header.stamp).nanoseconds) / 1e9
                        for p in candidates]
                quats = [(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
                         for p in candidates]
                orientation_spread = max(2*math.acos(min(1.0, abs(sum(x*y for x,y in zip(a,b)))))
                                         for a in quats for b in quats)
                if diameter <= tolerance and orientation_spread <= 0.08 and max(ages) <= float(self.get_parameter('detection_max_age_sec').value):
                    stable_pose = deepcopy(candidates[-1])
                    stable_pose.pose.position.x = median_xyz[0]
                    stable_pose.pose.position.y = median_xyz[1]
                    stable_pose.pose.position.z = median_xyz[2]
                    self.get_logger().info(
                        f"  Detection stabilized from {required} samples "
                        f"(max deviation {max_deviation:.3f} m, cluster diameter "
                        f"{diameter:.3f} m, limit {tolerance:.3f} m): "
                        + ", ".join(f"({p.pose.position.x:.3f}, {p.pose.position.y:.3f}, "
                                    f"{p.pose.position.z:.3f})" for p in candidates)
                    )
                    return stable_pose
            now = time.monotonic()
            if now >= next_progress_log:
                remaining = max(0.0, deadline - now)
                details = (
                    f"  Detection wait: {received_count} received, {usable_count} eligible; "
                    f"checking latest {min(usable_count, required)}/{required} samples"
                )
                if reference_pose is not None and max_refresh_shift > 0:
                    details += (
                        f", {rejected_by_reference} outside the "
                        f"{max_refresh_shift:.3f} m refresh gate"
                    )
                if latest_deviation is not None:
                    details += (
                        f", latest cluster diameter={latest_deviation:.3f} m "
                        f"(limit {tolerance:.3f} m), orientation spread={orientation_spread:.3f} rad "
                        f"(limit 0.080 rad), oldest sample={max(ages):.1f}s"
                    )
                details += f"; {remaining:.1f}s remaining"
                self.get_logger().info(details)
                next_progress_log = now + 2.0
            time.sleep(0.1)
        self.get_logger().warn(
            f"  Detection wait expired: {usable_count}/{required} usable sample(s), "
            f"{received_count} received, {rejected_by_reference} rejected by refresh gate"
        )
        return None

    def _refresh_detection(
        self,
        sequence_name: str,
        detection_topic: str,
        timeout_sec: float,
        reference_pose: Optional[PoseStamped] = None,
        required_samples: Optional[int] = None,
        stability_tolerance_m: Optional[float] = None,
        max_refresh_shift_m: Optional[float] = None,
    ) -> Optional[PoseStamped]:
        """Force a fresh target pose by clearing the cached one and waiting again."""
        self._ensure_detection_subscription(detection_topic)
        self._detection_after_ns = self.get_clock().now().nanoseconds
        with self._detection_lock:
            self._detection_poses.pop(detection_topic, None)
            self._detection_samples[detection_topic] = []

        self._publish_status(
            f"[{sequence_name}] Waiting for object detection on {detection_topic}..."
        )
        detected_pose = self._wait_for_detection(
            detection_topic,
            timeout_sec,
            reference_pose=reference_pose,
            required_samples=required_samples,
            stability_tolerance_m=stability_tolerance_m,
            max_refresh_shift_m=max_refresh_shift_m,
        )
        if detected_pose is None:
            return None

        p = detected_pose.pose.position
        if not all(math.isfinite(value) for value in (p.x, p.y, p.z)):
            self.get_logger().error("Detection contained a non-finite position")
            return None
        if not detected_pose.header.frame_id:
            self.get_logger().error("Detection did not specify a coordinate frame")
            return None
        self.get_logger().info(
            f"  Detection acquired: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
        return detected_pose

    def _build_hardcoded_detection_pose(self, sequence_cfg: Dict) -> Optional[PoseStamped]:
        pose_cfg = sequence_cfg.get("hardcoded_detection_pose")
        if not pose_cfg:
            return None

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = str(pose_cfg.get("frame_id", self.base_frame))
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        position = pose_cfg.get("position", [0.0, 0.0, 0.0])
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        rpy_deg = pose_cfg.get("rpy_deg", [0.0, 0.0, 0.0])
        roll, pitch, yaw = [math.radians(float(v)) for v in rpy_deg]
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        return pose_msg

    def _resolve_detected_pose(self, step: Dict, detected: PoseStamped) -> Optional[Dict]:
        """Return a copy of the step config with position filled from detection + offsets."""
        resolved = dict(step)
        target_frame = str(step.get("frame_id", self.base_frame))
        group_name = str(step.get("group_name", self.default_group_name))

        detected_for_planning = detected
        source_frame = detected.header.frame_id or target_frame
        if group_name in ("base_arm", "base_press_arm"):
            target_frame = self.base_frame
            if source_frame != target_frame:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time.from_msg(detected.header.stamp),
                        timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                    )
                    detected_for_planning = tf2_geometry_msgs.do_transform_pose_stamped(
                        detected,
                        tf_msg,
                    )
                    self.get_logger().info(
                        f"  Transformed detected target {source_frame} -> {target_frame}"
                    )
                except Exception as exc:
                    self.get_logger().error(
                        f"  Could not transform detected target {source_frame} -> "
                        f"{target_frame}: {exc}"
                    )
                    return None
        elif source_frame:
            target_frame = source_frame

        ox = float(step.get("offset_x", 0.0))
        oy = float(step.get("offset_y", 0.0))
        oz = float(step.get("offset_z", 0.0))
        if step.get('offset_frame') == 'target':
            q = detected_for_planning.pose.orientation
            # Quaternion-vector rotation without a numpy dependency.
            tx = 2 * (q.y * oz - q.z * oy)
            ty = 2 * (q.z * ox - q.x * oz)
            tz = 2 * (q.x * oy - q.y * ox)
            ox, oy, oz = (ox + q.w * tx + q.y * tz - q.z * ty,
                          oy + q.w * ty + q.z * tx - q.x * tz,
                          oz + q.w * tz + q.x * ty - q.y * tx)
        if step.get('orientation_from_detection', False):
            q = detected_for_planning.pose.orientation
            resolved['orientation'] = [q.x, q.y, q.z, q.w]
            resolved.pop('rpy_deg', None)
        resolved["position"] = [
            detected_for_planning.pose.position.x + ox,
            detected_for_planning.pose.position.y + oy,
            detected_for_planning.pose.position.z + oz,
        ]
        resolved["frame_id"] = target_frame
        return resolved

    # ------------------------------------------------------------------
    # Arduino execution tracking
    # ------------------------------------------------------------------
    def _arduino_ack_callback(self, msg: String) -> None:
        ack = msg.data.strip()
        fields = ack.split()
        if len(fields) < 2 or not fields[1].isdigit() or int(fields[1]) != self._command_id:
            return
        if ack.upper().startswith("DONE"):
            with self._arduino_result_lock:
                self._arduino_error_message = None
            self.get_logger().info(f"  Arduino DONE received: {ack}")
            self._apply_pending_joint_state()
            self._arduino_done_event.set()
        elif ack.upper().startswith("ERROR"):
            with self._arduino_result_lock:
                self._arduino_error_message = ack
            self.get_logger().error(f"  Arduino execution error: {ack}")
            # Wake the waiter immediately. It will inspect the stored result
            # and return failure rather than waiting for its host-side timeout.
            self._arduino_done_event.set()

    def _arm_arduino_wait(self) -> None:
        """Clear the previous terminal result before publishing a new queue."""
        with self._arduino_result_lock:
            self._arduino_error_message = None
        self._arduino_done_event.clear()
        self._command_id = (time.time_ns() // 1000) % (2**32 - 1) + 1
        self._clear_collision_debug()
        self._command_started = time.monotonic()
        self._endpoint_profile = "precise"
        self._command_expected_duration = 0.0

    def _queue_pending_joint_state(self, positions: List[float], reason: str) -> None:
        with self._pending_joint_state_lock:
            self._pending_joint_state = list(positions)
            self._pending_joint_state_reason = reason

    def _apply_pending_joint_state(self) -> None:
        # Commands are expectations for completion checking, never measurements.
        return

    def _force_apply_pending_joint_state(self, reason: str) -> None:
        self.stop_pub.publish(Bool(data=True))
        with self._pending_joint_state_lock:
            self._pending_joint_state = None
        self.get_logger().error(f"Execution stopped ({reason}); measured state retained")

    def _wait_for_arduino_done(self, timeout_sec: float, label: str = "Arduino command") -> bool:
        """Block until Arduino sends DONE, or timeout.

        The event MUST be cleared by the caller before publishing the command
        queue — otherwise a fast DONE that arrives before this wait starts will
        be wiped out by a clear() here.
        """
        timeout_sec = max(timeout_sec, getattr(self, "_command_expected_duration", 0.0) + 15.0)
        start_time = time.monotonic()
        next_log = start_time
        progress_period = max(0.5, self.arduino_wait_progress_sec)

        while True:
            elapsed = time.monotonic() - start_time
            remaining = timeout_sec - elapsed
            if remaining <= 0.0:
                self.stop_pub.publish(Bool(data=True))
                self.get_logger().warn(
                    f"{label}: Arduino did not send DONE within {timeout_sec:.1f}s"
                )
                return False

            if self._arduino_done_event.wait(timeout=min(0.2, remaining)):
                elapsed = time.monotonic() - start_time
                with self._arduino_result_lock:
                    error_message = self._arduino_error_message
                if error_message is not None:
                    self.get_logger().error(
                        f"{label}: Arduino failed after {elapsed:.1f}s: "
                        f"{error_message}"
                    )
                    return False
                # DDS may deliver DONE before the preceding joint-state message.
                until = time.monotonic() + 1.5
                while time.monotonic() < until:
                    if self._measured_goal_reached():
                        self.get_logger().info(f"{label}: measured endpoint verified")
                        return True
                    time.sleep(0.02)
                self.get_logger().error(f"{label}: DONE without a settled measured endpoint")
                self.stop_pub.publish(Bool(data=True))
                return False

            now = time.monotonic()
            if now >= next_log:
                elapsed = now - start_time
                self.get_logger().info(
                    f"{label}: waiting for Arduino DONE ({elapsed:.1f}/{timeout_sec:.1f}s)"
                )
                next_log = now + progress_period

    # ------------------------------------------------------------------
    # Trajectory approval
    # ------------------------------------------------------------------
    def _trajectory_approval_callback(self, msg: Bool) -> None:
        with self._plan_lock:
            active_plan = self._active_plan
            accepting_response = bool(
                active_plan
                and not active_plan.cancelled
                and active_plan.awaiting_approval
            )
        if not accepting_response:
            self.get_logger().warn("Ignoring trajectory approval: no plan is awaiting approval")
            return

        with self._trajectory_approval_lock:
            self._trajectory_approval_response = bool(msg.data)
            self._trajectory_approval_event.set()

        decision = "approved" if msg.data else "rejected"
        self.get_logger().info(f"Trajectory approval response received: {decision}")

    def _reset_trajectory_approval(self) -> None:
        with self._trajectory_approval_lock:
            self._trajectory_approval_response = None
            self._trajectory_approval_event.clear()

    def _wait_for_trajectory_approval(self, name: str, context: PlanContext) -> bool:
        timeout_sec = self.trajectory_approval_timeout_sec
        approve_cmd = (
            "ros2 topic pub --once "
            f"{self.trajectory_approval_topic} std_msgs/msg/Bool \"{{data: true}}\""
        )
        reject_cmd = (
            "ros2 topic pub --once "
            f"{self.trajectory_approval_topic} std_msgs/msg/Bool \"{{data: false}}\""
        )

        self._publish_status(f"Plan for {name} is waiting for approval")
        self.get_logger().info(f"Plan for {name} is displayed in RViz.")
        self.get_logger().info(f"  Approve: {approve_cmd}")
        self.get_logger().info(f"  Reject:  {reject_cmd}")

        start_time = time.monotonic()
        next_log = start_time
        progress_period = 5.0

        while True:
            if context.cancelled:
                self.get_logger().warn(f"Approval wait cancelled for {name}")
                return False
            if timeout_sec > 0.0:
                elapsed = time.monotonic() - start_time
                remaining = timeout_sec - elapsed
                if remaining <= 0.0:
                    self.get_logger().warn(
                        f"Plan approval timed out for {name} after {timeout_sec:.1f}s"
                    )
                    return False
                wait_sec = min(0.2, remaining)
            else:
                wait_sec = 0.2

            if self._trajectory_approval_event.wait(timeout=wait_sec):
                with self._trajectory_approval_lock:
                    approved = bool(self._trajectory_approval_response)
                if approved:
                    self.get_logger().info(f"Plan for {name} approved; sending to Arduino")
                else:
                    self.get_logger().warn(f"Plan for {name} rejected; not sending to Arduino")
                return approved

            now = time.monotonic()
            if now >= next_log:
                if timeout_sec > 0.0:
                    elapsed = now - start_time
                    self.get_logger().info(
                        f"Plan for {name} awaiting approval ({elapsed:.1f}/{timeout_sec:.1f}s)"
                    )
                else:
                    self.get_logger().info(f"Plan for {name} awaiting approval")
                next_log = now + progress_period

    def _plan_wait_timeout(self, command_cfg: Dict) -> Optional[float]:
        planning_time = float(command_cfg.get("allowed_planning_time", self.allowed_planning_time))
        require_approval = bool(
            command_cfg.get("require_trajectory_approval", self.require_trajectory_approval)
        )
        if not require_approval:
            return max(180.0, planning_time + 10.0)

        approval_time = self.trajectory_approval_timeout_sec
        if approval_time <= 0.0:
            return None
        return max(180.0, planning_time + approval_time + 10.0)

    def _publish_display_trajectory(self, name: str, result) -> None:
        trajectory = result.planned_trajectory
        if not trajectory or not trajectory.joint_trajectory.points:
            return

        msg = DisplayTrajectory()
        msg.model_id = "homeplus_urdf"
        if hasattr(result, "trajectory_start"):
            msg.trajectory_start = result.trajectory_start
        elif self.latest_joint_state is not None:
            msg.trajectory_start.joint_state = deepcopy(self.latest_joint_state)
        msg.trajectory = [trajectory]

        # Publish a few times so an already-open RViz reliably catches it.
        for _ in range(3):
            self.display_trajectory_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info(f"Published planned path for {name} to RViz")

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------
    def _load_execution_map(self, path: str) -> Dict[str, Dict]:
        with open(path, "r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        commands = payload.get("commands", {})
        if not commands:
            raise ValueError(f"No commands found in {path}")
        errors = []
        for name, command_cfg in commands.items():
            errors.extend(self._command_config_errors(str(name), command_cfg))
        if errors:
            formatted = "\n  - ".join(errors)
            raise ValueError(f"Invalid execution map {path}:\n  - {formatted}")
        return commands

    @staticmethod
    def _command_config_errors(name: str, command_cfg: Dict) -> List[str]:
        errors: List[str] = []
        if not isinstance(command_cfg, dict):
            return [f"{name}: command must be a mapping"]

        allowed = set("endpoint_profile name type group_name joints value frame_id position orientation rpy_deg target steps task_id detection_topic detection_timeout_sec detection_min_stable_samples detection_stability_tolerance_m detection_max_refresh_shift_m max_replanning_attempts max_attempts settle_sec require_trajectory_approval planner_id allowed_planning_time num_planning_attempts link_name offset_x offset_y offset_z offset_frame orientation_from_detection gripper_value final_gripper_value position_tolerance_m position_tolerance_xy_m position_tolerance_z_m use_orientation_constraint orientation_tolerance_rad orientation_tolerance_x_rad orientation_tolerance_y_rad orientation_tolerance_z_rad safe_z_candidate_offsets_m base_goal_tcp_x_min_m base_goal_tcp_x_max_m base_goal_tcp_y_max_m max_base_goal_rotation_rad max_base_goal_translation_m max_base_goal_y_translation_m preserve_reference_z maintain_orientation_during_path post_delay_sec cartesian mobile_standoff lookahead_offsets velocity_scaling acceleration_scaling clear_octomap_before_plan octomap_clear_timeout_sec octomap_rebuild_wait_sec ik_candidate_attempts attach_cup joint_tolerance hardcoded_detection_pose hardcoded_detection_frame planning_service_timeout_sec phase_allowed_planning_time phase_num_planning_attempts phase_joint_tolerance continue_on_arduino_timeout base_placement heading_from_base max_arm_plane_error_m redo_from_step max_redos".split())
        if command_cfg.get('endpoint_profile', 'precise') not in ('precise', 'transit'):
            errors.append(f'{name}: endpoint_profile must be precise or transit')
        if command_cfg.get('cartesian') and command_cfg.get('endpoint_profile') == 'transit':
            errors.append(f'{name}: contact Cartesian plans require the precise endpoint profile')
        if 'detection_max_refresh_shift_m' in command_cfg:
            shift = float(command_cfg['detection_max_refresh_shift_m'])
            if not math.isfinite(shift) or shift < 0:
                errors.append(f'{name}: detection_max_refresh_shift_m must be nonnegative (zero disables it)')
        unknown = set(command_cfg) - allowed
        if unknown:
            errors.append(f'{name}: unsupported configuration keys: {sorted(unknown)}')
        if command_cfg.get('cartesian') and command_cfg.get('group_name') not in ('arm', 'press_arm'):
            errors.append(f'{name}: contact Cartesian plans require a fixed-base arm group')
        for key in ('velocity_scaling', 'acceleration_scaling'):
            if key in command_cfg and not 0 < float(command_cfg[key]) <= 1:
                errors.append(f'{name}: {key} must be in (0, 1]')
        action_type = str(command_cfg.get("type", "command"))
        supported = {"pose", "joint_state", "gripper", "command", "sequence", "detect"}
        if action_type not in supported:
            errors.append(f"{name}: unsupported type '{action_type}'")

        if action_type == "pose":
            position = command_cfg.get("position")
            target = command_cfg.get("target")
            supported_pose_targets = {"detected", "previous_pose"}
            if target is not None and target not in supported_pose_targets:
                errors.append(
                    f"{name}: unsupported pose target '{target}'"
                )
            if target not in supported_pose_targets and position is None:
                errors.append(
                    f"{name}: pose requires position, target: detected, "
                    "or target: previous_pose"
                )
            if position is not None and (
                not isinstance(position, list)
                or len(position) != 3
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in position)
            ):
                errors.append(f"{name}: position must contain three finite numbers")
            rpy = command_cfg.get("rpy_deg")
            orientation = command_cfg.get("orientation")
            if rpy is not None and (not isinstance(rpy, list) or len(rpy) != 3):
                errors.append(f"{name}: rpy_deg must contain three numbers")
            if orientation is not None and (
                not isinstance(orientation, list) or len(orientation) != 4
            ):
                errors.append(f"{name}: orientation must contain four numbers")

        placement = command_cfg.get('base_placement')
        if placement is not None:
            allowed_placement = {'heading_offsets_deg', 'min_reach_margin_m',
                                 'joint_limit_margin_rad', 'base_motion', 'reach_preference'}
            if not isinstance(placement, dict):
                errors.append(f'{name}: base_placement must be a mapping')
            else:
                if set(placement) - allowed_placement:
                    errors.append(f'{name}: unsupported base_placement keys: '
                                  f'{sorted(set(placement) - allowed_placement)}')
                if placement.get('base_motion', 'combined') not in ('combined', 'turn_then_drive'):
                    errors.append(f'{name}: base_placement.base_motion must be combined or turn_then_drive')
                if placement.get('reach_preference', 'centre') not in ('centre', 'near'):
                    errors.append(f'{name}: base_placement.reach_preference must be centre or near')
                for key in ('min_reach_margin_m', 'joint_limit_margin_rad'):
                    if key in placement and float(placement[key]) < 0.0:
                        errors.append(f'{name}: base_placement.{key} must be nonnegative')
            if not command_cfg.get('mobile_standoff') or command_cfg.get('group_name') not in ('base_arm', 'base_press_arm'):
                errors.append(f'{name}: base_placement requires mobile_standoff and a mobile-base group')
            if command_cfg.get('target') != 'detected':
                errors.append(f'{name}: base_placement requires target: detected')
            if any(k in command_cfg for k in ('base_goal_tcp_x_min_m', 'base_goal_tcp_x_max_m', 'base_goal_tcp_y_max_m')):
                errors.append(f'{name}: base_placement computes the base pose; remove base_goal_tcp_* bounds')
        if command_cfg.get('heading_from_base'):
            if command_cfg.get('group_name') not in ('arm', 'press_arm') or command_cfg.get('target') != 'detected':
                errors.append(f'{name}: heading_from_base requires a fixed-base arm group and target: detected')
        if 'max_arm_plane_error_m' in command_cfg:
            if action_type != 'detect' or float(command_cfg['max_arm_plane_error_m']) <= 0.0:
                errors.append(f'{name}: max_arm_plane_error_m must be positive on a detect step')
            if not command_cfg.get('redo_from_step'):
                errors.append(f'{name}: max_arm_plane_error_m requires redo_from_step')
        if 'max_redos' in command_cfg:
            value = command_cfg['max_redos']
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                errors.append(f'{name}: max_redos must be a nonnegative integer')

        if action_type == "joint_state" and not command_cfg.get("joints"):
            errors.append(f"{name}: joint_state requires a non-empty joints mapping")

        if action_type == "gripper":
            value = command_cfg.get("value")
            if not isinstance(value, (int, float)) or not 0.0 <= float(value) <= 90.0:
                errors.append(f"{name}: gripper value must be in Arduino degrees [0, 90]")
            max_attempts = command_cfg.get("max_attempts", 1)
            if not isinstance(max_attempts, int) or max_attempts < 1:
                errors.append(f"{name}: max_attempts must be a positive integer")

        for key in (
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "orientation_tolerance_x_rad",
            "orientation_tolerance_y_rad",
            "orientation_tolerance_z_rad",
            "joint_tolerance",
            "max_base_goal_translation_m",
            "max_base_goal_y_translation_m",
            "base_goal_tcp_y_max_m",
            "max_base_goal_rotation_rad",
            "base_goal_tcp_x_min_m",
            "base_goal_tcp_x_max_m",
            "detection_stability_tolerance_m",
        ):
            if key in command_cfg and float(command_cfg[key]) <= 0.0:
                errors.append(f"{name}: {key} must be positive")
        reach_min = command_cfg.get("base_goal_tcp_x_min_m")
        reach_max = command_cfg.get("base_goal_tcp_x_max_m")
        if (reach_min is None) != (reach_max is None):
            errors.append(
                f"{name}: base_goal_tcp_x_min_m and "
                "base_goal_tcp_x_max_m must be set together"
            )
        elif reach_min is not None and float(reach_min) >= float(reach_max):
            errors.append(
                f"{name}: base_goal_tcp_x_min_m must be less than "
                "base_goal_tcp_x_max_m"
            )
        for key in ("allowed_planning_time", "num_planning_attempts"):
            if key in command_cfg and float(command_cfg[key]) <= 0.0:
                errors.append(f"{name}: {key} must be positive")
        if "max_replanning_attempts" in command_cfg:
            value = command_cfg["max_replanning_attempts"]
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                errors.append(
                    f"{name}: max_replanning_attempts must be a positive integer"
                )

        if action_type == "sequence":
            steps = command_cfg.get("steps")
            if not isinstance(steps, list) or not steps:
                errors.append(f"{name}: sequence requires a non-empty steps list")
            else:
                for index, step in enumerate(steps):
                    step_name = str(step.get("name", f"step_{index}")) if isinstance(step, dict) else f"step_{index}"
                    errors.extend(
                        GestureMotionExecutor._command_config_errors(
                            f"{name}/{step_name}", step
                        )
                    )
                names = [str(step.get("name", "")) if isinstance(step, dict) else "" for step in steps]
                for index, step in enumerate(steps):
                    if isinstance(step, dict) and step.get("redo_from_step") is not None:
                        if step["redo_from_step"] not in names[:index]:
                            errors.append(
                                f"{name}/{names[index]}: redo_from_step must name an earlier step"
                            )
                uses_detection = any(
                    isinstance(step, dict)
                    and (step.get("target") == "detected" or step.get("type") == "detect")
                    for step in steps
                )
                if uses_detection and not (
                    command_cfg.get("detection_topic")
                    or command_cfg.get("hardcoded_detection_pose")
                ):
                    errors.append(f"{name}: detected steps require detection_topic")
        return errors

    def _ready_callback(self, msg):
        self._hardware_ready = msg.data

    def _cloud_ready_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['masked']:
                self._cloud_stamp_ns = int(data['stamp_ns'])
        except (ValueError, KeyError, TypeError):
            pass

    def joint_states_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        if not set(TRAJECTORY_JOINT_ORDER).issubset(msg.name):
            return
        if not all(math.isfinite(v) for v in msg.position):
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age < -0.1 or age > float(self.get_parameter('joint_state_max_age_sec').value):
            return
        msg = deepcopy(msg)
        if self.dry_run and self._virtual_positions:
            msg.position = [self._virtual_positions.get(n, p) for n, p in zip(msg.name, msg.position)]
        self.latest_joint_state = msg
        self._last_state_received = time.monotonic()
        self._joint_history.append((self._last_state_received, deepcopy(msg)))

    def _require_fresh_state(self):
        if self.latest_joint_state is None or time.monotonic() - self._last_state_received > 0.5:
            raise RuntimeError('No fresh hardware joint feedback; refusing to plan or execute')

    def _measured_goal_reached(self):
        if self.latest_joint_state is None or time.monotonic() - self._last_state_received > 0.3:
            return False
        target = self._pending_joint_state
        if target is None:
            return False
        recent = [(t, js) for t, js in list(self._joint_history)
                  if t >= max(self._command_started, time.monotonic() - 0.7)]
        if len(recent) < 3 or recent[-1][0] - recent[0][0] < 0.35:
            return False
        for _, js in recent:
            q = dict(zip(js.name, js.position))
            for i, name in enumerate(TRAJECTORY_JOINT_ORDER):
                tolerance = 0.003 if name in ('base_x', 'base_y') else math.radians(0.6)
                if getattr(self, '_endpoint_profile', 'precise') == 'transit':
                    tolerance = 0.0051 if name in ('base_x', 'base_y') else math.radians(1.1)
                # PWM-only joints have coarser estimation; never claim grasp verification.
                if name in ('joint_grip_L', 'joint_hand', 'joint_wrist'):
                    tolerance = math.radians(3.0)
                if abs(q[name] - target[i]) > tolerance:
                    return False
        return True

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------
    def command_callback(self, msg: GestureCommand) -> None:
        if not self._try_set_busy():
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
            self._set_busy(False)
            self._publish_status(f"No execution configured for command {command_name}")
            self.get_logger().warn(f"No execution configured for command {command_name}")
            return

        # All command types use the same serialized worker path. Previously,
        # only sequences claimed the busy flag, so two single-pose commands
        # could share approval/events and execute the wrong plan.
        thread = threading.Thread(
            target=self._run_command,
            args=(command_name, command_cfg, msg.command_type),
            daemon=True,
        )
        thread.start()

    def _run_command(self, command_name: str, command_cfg: Dict, fallback_type: str) -> None:
        try:
            self._require_fresh_state()
            if not self._hardware_ready:
                raise RuntimeError('Hardware not ready; protocol-v2 feedback and a healthy bridge are required')
            if self.enable_moveit_planning:
                scene = GetPlanningScene.Request()
                scene.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                attached = self._call_service(self.scene_client, scene).scene.robot_state.attached_collision_objects
                self._holding_cup = any(o.object.id == 'detected_cup' for o in attached)
            action_type = str(command_cfg.get("type", fallback_type or "command"))
            if action_type == "sequence":
                self._run_sequence(command_name, command_cfg)
            else:
                self._execute_single_step(command_name, command_cfg, fallback_type)
        except Exception as exc:
            self.stop_pub.publish(Bool(data=True))
            if self._active_plan is not None:
                self._cancel_plan(self._active_plan, str(exc))
            self.get_logger().error(f"Command {command_name} crashed: {exc}")
            self._publish_status(f"Command {command_name} FAILED: {exc}")
        finally:
            self._set_busy(False)

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
                context = self._plan_pose(command_name, pose_msg, command_cfg)
                plan_succeeded = bool(
                    context is not None and self._wait_for_plan(context, command_cfg)
                )
                if not plan_succeeded:
                    self._publish_status(f"Command {command_name} failed during planning")
                else:
                    self._complete_single_trajectory(command_name, command_cfg)
        elif action_type == "joint_state":
            joint_msg = self._build_joint_state_message(command_cfg)
            self.joint_target_pub.publish(joint_msg)
            if self.enable_moveit_planning:
                context = self._plan_joint_state(command_name, joint_msg, command_cfg)
                plan_succeeded = bool(
                    context is not None and self._wait_for_plan(context, command_cfg)
                )
                if not plan_succeeded:
                    self._publish_status(f"Command {command_name} failed during planning")
                else:
                    self._complete_single_trajectory(command_name, command_cfg)
        elif action_type == "gripper":
            grip_val = float(command_cfg["value"])
            self.gripper_target_pub.publish(Float32(data=grip_val))
            max_attempts = int(command_cfg.get("max_attempts", 1))
            for attempt in range(1, max_attempts + 1):
                self._send_gripper_to_arduino(grip_val)
                if self.wait_for_gripper_ack:
                    if not self._wait_for_arduino_done(
                        15.0, f"{command_name} gripper"
                    ):
                        self._force_apply_pending_joint_state("gripper timeout")
                        if attempt < max_attempts:
                            continue
                        self._publish_status(
                            f"Command {command_name} failed: gripper timeout"
                        )
                        break
                else:
                    self._force_apply_pending_joint_state(
                        "gripper acknowledgement disabled"
                    )
                if grip_val <= 1.0 and self._holding_cup:
                    self._detach_cup()
                settle_sec = float(command_cfg.get("settle_sec", self.gripper_settle_sec))
                if settle_sec > 0.0:
                    time.sleep(settle_sec)
                break
        elif action_type == "command":
            pass
        else:
            self.get_logger().warn(
                f"Unsupported execution type '{action_type}' for {command_name}"
            )

    def _complete_single_trajectory(self, command_name: str, command_cfg: Dict) -> None:
        if self.wait_for_trajectory_ack:
            if not self._wait_for_arduino_done(60.0, f"{command_name} trajectory"):
                self._force_apply_pending_joint_state(self._arduino_error_message or "trajectory acknowledgement/endpoint verification failed")
                self._publish_status(f"Command {command_name} failed: trajectory timeout")
        else:
            self._force_apply_pending_joint_state("trajectory acknowledgement disabled")

    # ------------------------------------------------------------------
    # Sequence execution
    # ------------------------------------------------------------------
    def _run_sequence(self, sequence_name: str, sequence_cfg: Dict) -> None:
        if self._holding_cup:
            raise RuntimeError('Release the attached cup before starting another manipulation task')
        steps: List[Dict] = sequence_cfg.get("steps", [])
        if not steps:
            self.get_logger().error(f"Sequence {sequence_name} has no steps")
            return

        self._start_sequence_trajectory_log(sequence_name)
        self._publish_status(f"Starting sequence: {sequence_name} ({len(steps)} steps)")
        self.get_logger().info(
            f"=== Sequence '{sequence_name}' started ({len(steps)} steps) ==="
        )

        # If the sequence uses detection, subscribe and wait for a pose unless
        # a temporary hardcoded test pose is configured.
        detection_topic = sequence_cfg.get("detection_topic")
        detected_pose: Optional[PoseStamped] = self._build_hardcoded_detection_pose(sequence_cfg)
        needs_detection = detection_topic and any(
            s.get("target") == "detected" for s in steps
        )

        if 'task_id' in sequence_cfg:
            req = SetParameters.Request()
            req.parameters = [Parameter('task_id', value=int(sequence_cfg['task_id'])).to_parameter_msg()]
            response = self._call_service(self.detector_config_client, req)
            if not response.results or not all(r.successful for r in response.results):
                raise RuntimeError('Detector rejected requested manipulation task')

        if detected_pose is not None:
            p = detected_pose.pose.position
            self.get_logger().warn(
                f"[{sequence_name}] Using hardcoded detection pose in "
                f"{detected_pose.header.frame_id}: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
            )
        elif needs_detection:
            timeout = float(sequence_cfg.get("detection_timeout_sec", 5.0))
            detected_pose = self._refresh_detection(
                sequence_name,
                detection_topic,
                timeout,
                required_samples=int(
                    sequence_cfg.get(
                        "detection_min_stable_samples",
                        self.detection_min_stable_samples,
                    )
                ),
                stability_tolerance_m=sequence_cfg.get("detection_stability_tolerance_m"),
            )
            if detected_pose is None:
                self._abort_sequence(
                    sequence_name,
                    "detection",
                    f"no stable detection on {detection_topic} within {timeout}s",
                )
                return

        if int(sequence_cfg.get('task_id', 0)) == 1:
            if self._holding_cup:
                raise RuntimeError('Cup remains attached; explicitly release/remove it before another pickup')
            self._update_cup_scene(detected_pose)
        last_successful_pose: Optional[PoseStamped] = None
        redos: Dict[str, int] = {}
        # A while loop so a detect step can send the sequence back to an
        # earlier step (re-place the base from a better observation).
        i = -1
        while i + 1 < len(steps):
            i += 1
            step = steps[i]
            step_name = step.get("name", f"step_{i}")
            step_type = step.get("type", "command")

            # Resolve detected positions
            effective_step = step
            if step.get("target") == "detected" and detected_pose is not None:
                effective_step = self._resolve_detected_pose(step, detected_pose)
                if effective_step is None:
                    self._abort_sequence(
                        sequence_name,
                        step_name,
                        "could not transform detected pose into the planning frame",
                    )
                    return
            elif step.get("target") == "previous_pose":
                if last_successful_pose is None:
                    self._abort_sequence(
                        sequence_name,
                        step_name,
                        "previous pose is unavailable",
                    )
                    return
                effective_step = self._resolve_detected_pose(
                    step, last_successful_pose
                )
                if effective_step is None:
                    self._abort_sequence(
                        sequence_name,
                        step_name,
                        "could not resolve previous pose in the planning frame",
                    )
                    return

            if step.get("heading_from_base") and detected_pose is not None:
                effective_step = self._resolve_in_arm_plane(effective_step, detected_pose)
                if effective_step is None:
                    self._abort_sequence(sequence_name, step_name,
                                         "target is outside the arm's reach from the measured base")
                    return

            if step.get("target") == "detected" and detected_pose is not None:
                pos = effective_step["position"]
                frame_id = effective_step.get("frame_id", self.base_frame)
                self.get_logger().info(
                    f"  {step_name}: resolved target in {frame_id} -> "
                    f"[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
                )
                self._append_target_audit(
                    sequence_name=sequence_name,
                    step_name=step_name,
                    detection_topic=str(detection_topic or ""),
                    detected_pose=detected_pose,
                    resolved_step=effective_step,
                )
            elif step.get("target") == "previous_pose":
                pos = effective_step["position"]
                frame_id = effective_step.get("frame_id", self.base_frame)
                self.get_logger().info(
                    f"  {step_name}: resolved from previous successful pose in "
                    f"{frame_id} -> [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
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
                max_attempts = int(effective_step.get("max_attempts", 1))
                settle_sec = float(
                    effective_step.get("settle_sec", self.gripper_settle_sec)
                )
                for attempt in range(1, max_attempts + 1):
                    if max_attempts > 1:
                        self.get_logger().info(
                            f"  Gripper command attempt {attempt}/{max_attempts}"
                        )
                    self._send_gripper_to_arduino(grip_val)
                    if self.wait_for_gripper_ack:
                        self.get_logger().info("  Waiting for Arduino to finish gripper...")
                        if not self._wait_for_arduino_done(
                            timeout_sec=15.0,
                            label=f"{sequence_name}/{step_name} gripper",
                        ):
                            self._force_apply_pending_joint_state("gripper timeout")
                            if attempt < max_attempts:
                                self.get_logger().warn(
                                    f"  Retrying gripper command for {step_name}"
                                )
                                continue
                            if not bool(effective_step.get(
                                "continue_on_arduino_timeout",
                                self.continue_on_arduino_timeout,
                            )):
                                self._abort_sequence(
                                    sequence_name, step_name, "gripper timeout"
                                )
                                return
                            self.get_logger().warn(
                                f"  Continuing after gripper timeout for {step_name}"
                            )
                    else:
                        self.get_logger().info(
                            "  Continuing without gripper DONE acknowledgement"
                        )
                        self._force_apply_pending_joint_state(
                            "gripper acknowledgement disabled"
                        )

                    if settle_sec > 0.0:
                        self.get_logger().info(
                            f"  Allowing gripper to settle for {settle_sec:.1f}s"
                        )
                        time.sleep(settle_sec)
                    if effective_step.get('attach_cup', False):
                        if self.dry_run:
                            # Never leave a phantom attached cup in the live scene.
                            self.get_logger().warn('DRY RUN: not attaching the cup collision object')
                        else:
                            self._attach_cup()
                    break

            elif step_type == "detect":
                if not detection_topic:
                    self._abort_sequence(sequence_name, step_name, "no detection_topic configured")
                    return

                timeout = float(effective_step.get(
                    "detection_timeout_sec",
                    sequence_cfg.get("detection_timeout_sec", 5.0),
                ))
                refreshed_pose = self._refresh_detection(
                    sequence_name,
                    detection_topic,
                    timeout,
                    reference_pose=detected_pose,
                    # Step keys override the sequence's, which override the node's.
                    required_samples=int(
                        effective_step.get(
                            "detection_min_stable_samples",
                            sequence_cfg.get(
                                "detection_min_stable_samples",
                                self.detection_min_stable_samples,
                            ),
                        )
                    ),
                    stability_tolerance_m=effective_step.get(
                        "detection_stability_tolerance_m",
                        sequence_cfg.get("detection_stability_tolerance_m"),
                    ),
                    max_refresh_shift_m=(
                        float(effective_step["detection_max_refresh_shift_m"])
                        if "detection_max_refresh_shift_m" in effective_step
                        else None
                    ),
                )
                if (
                    refreshed_pose is not None
                    and detected_pose is not None
                    and bool(effective_step.get("preserve_reference_z", False))
                ):
                    measured_z = refreshed_pose.pose.position.z
                    reference_z = detected_pose.pose.position.z
                    refreshed_pose.pose.position.z = reference_z
                    self.get_logger().info(
                        f"  Refined X/Y while preserving target Z={reference_z:.3f} m "
                        f"(new Z measurement was {measured_z:.3f} m)"
                    )
                if refreshed_pose is not None:
                    detected_pose = refreshed_pose
                    if int(sequence_cfg.get('task_id', 0)) == 1:
                        self._update_cup_scene(detected_pose)
                if refreshed_pose is None:
                    self._abort_sequence(
                        sequence_name,
                        step_name,
                        f"no stable detection on {detection_topic} within {timeout}s",
                    )
                    return

                limit = effective_step.get("max_arm_plane_error_m")
                if limit is not None:
                    # The fixed-base steps cannot move the gripper sideways;
                    # re-place the base rather than grasp off-centre.
                    plane = self._arm_plane_offset(detected_pose, effective_step.get("link_name"))
                    if plane is None:
                        self._abort_sequence(sequence_name, step_name, "arm reach model unavailable")
                        return
                    error, forward = plane
                    reachable = self._arm_only_reachable(steps, i, detected_pose, forward)
                    self.get_logger().info(
                        f"  {step_name}: target {error * 1000:+.1f} mm from the arm plane "
                        f"(limit +/-{float(limit) * 1000:.0f} mm), {forward:.3f} m ahead of base_link; "
                        f"remaining arm-only waypoints {'reachable' if reachable else 'NOT reachable'}"
                    )
                    if abs(error) > float(limit) or not reachable:
                        target_step = effective_step["redo_from_step"]
                        done = redos.get(step_name, 0)
                        allowed = int(effective_step.get("max_redos", 1))
                        if done >= allowed:
                            self._abort_sequence(
                                sequence_name, step_name,
                                f"target {error * 1000:+.1f} mm off the arm plane"
                                f"{'' if reachable else ', out of arm-only reach'} after "
                                f"{done} re-placement(s)")
                            return
                        redos[step_name] = done + 1
                        self.get_logger().warn(
                            f"  {step_name}: re-placing the base from the close-up detection; "
                            f"redoing from {target_step} ({done + 1}/{allowed})")
                        i = [str(st.get("name", "")) for st in steps].index(target_step) - 1
                        continue

            elif step_type == "pose":
                if not self.enable_moveit_planning or self.move_group_client is None:
                    self._abort_sequence(sequence_name, step_name, "MoveIt not enabled")
                    return

                pose_msg = self._build_pose_message(effective_step)
                if effective_step.get("base_placement"):
                    candidate_pose = self._plan_with_base_placement(
                        step_name, effective_step, detected_pose, steps, i)
                    planning_succeeded = candidate_pose is not None
                else:
                    candidate_z_offsets = [
                        float(value)
                        for value in effective_step.get(
                            "safe_z_candidate_offsets_m", [0.0]
                        )
                    ]
                    if not candidate_z_offsets:
                        candidate_z_offsets = [0.0]

                    max_replanning_attempts = max(
                        1,
                        int(
                            effective_step.get(
                                "max_replanning_attempts",
                                sequence_cfg.get("max_replanning_attempts", 1),
                            )
                        ),
                    )
                    planning_succeeded = False
                    for planning_attempt in range(1, max_replanning_attempts + 1):
                        candidate_index = min(
                            planning_attempt - 1, len(candidate_z_offsets) - 1
                        )
                        candidate_z_offset = candidate_z_offsets[candidate_index]
                        candidate_pose = deepcopy(pose_msg)
                        candidate_pose.pose.position.z += candidate_z_offset
                        self.target_pose_pub.publish(candidate_pose)
                        if max_replanning_attempts > 1:
                            self.get_logger().info(
                                f"  Planning {step_name}: outer attempt "
                                f"{planning_attempt}/{max_replanning_attempts}, "
                                f"safe candidate z={candidate_pose.pose.position.z:.3f} m "
                                f"(offset +{candidate_z_offset:.3f} m)"
                            )
                        attempt_step = effective_step
                        if planning_attempt > 1:
                            # The first attempt already cleared the map and waited
                            # for a fresh cloud. Rebuilding the same scene on every
                            # stochastic OMPL retry adds latency and can reinsert
                            # the same target-edge noise that just caused failure.
                            attempt_step = dict(effective_step)
                            attempt_step["clear_octomap_before_plan"] = False
                            self.get_logger().info(
                                f"  Reusing prepared OctoMap for {step_name} retry"
                            )
                        context = self._plan_pose(
                            step_name, candidate_pose, attempt_step
                        )
                        if context is not None and self._wait_for_plan(context, attempt_step):
                            planning_succeeded = True
                            break

                        retryable = context is None or context.retryable_failure
                        if not retryable or planning_attempt >= max_replanning_attempts:
                            break
                        self.get_logger().warn(
                            f"  Replanning {step_name} after MoveIt failure "
                            f"({planning_attempt}/{max_replanning_attempts})"
                        )
                        time.sleep(0.2)

                if not planning_succeeded:
                    self._abort_sequence(sequence_name, step_name, "planning/execution failed")
                    return

                if (
                    step.get("target") == "detected"
                    and detected_pose is not None
                ):
                    selected_step = dict(effective_step)
                    selected_step["position"] = [
                        candidate_pose.pose.position.x,
                        candidate_pose.pose.position.y,
                        candidate_pose.pose.position.z,
                    ]
                    selected_step["_unclamped_position_z_m"] = float(
                        effective_step.get(
                            "_unclamped_position_z_m",
                            effective_step["position"][2],
                        )
                    )
                    self._append_target_audit(
                        sequence_name=sequence_name,
                        step_name=f"{step_name}_selected_candidate",
                        detection_topic=str(detection_topic or ""),
                        detected_pose=detected_pose,
                        resolved_step=selected_step,
                    )

                if self.wait_for_trajectory_ack:
                    self.get_logger().info("  Waiting for Arduino to finish trajectory...")
                    if not self._wait_for_arduino_done(
                        timeout_sec=60.0,
                        label=f"{sequence_name}/{step_name} trajectory",
                    ):
                        self._force_apply_pending_joint_state(self._arduino_error_message or "trajectory acknowledgement/endpoint verification failed")
                        if not bool(effective_step.get(
                            "continue_on_arduino_timeout",
                            self.continue_on_arduino_timeout,
                        )):
                            self._abort_sequence(sequence_name, step_name, self._arduino_error_message or "trajectory acknowledgement/endpoint verification failed")
                            return
                        self.get_logger().warn(f"  Continuing after trajectory timeout for {step_name}")
                else:
                    self.get_logger().info("  Continuing without trajectory DONE acknowledgement")
                    self._force_apply_pending_joint_state(
                        "trajectory acknowledgement disabled"
                    )

                last_successful_pose = self._fk_pose(self.latest_joint_state,
                    effective_step.get('link_name', self.end_effector_link), candidate_pose.header.frame_id)
                if last_successful_pose is None:
                    self._abort_sequence(sequence_name, step_name, 'reached TCP FK unavailable')
                    return

                post_delay_sec = float(effective_step.get("post_delay_sec", 0.0))
                if post_delay_sec > 0.0:
                    self.get_logger().info(
                        f"  Waiting {post_delay_sec:.1f}s after {step_name} before next step"
                    )
                    time.sleep(post_delay_sec)

            elif step_type == "joint_state":
                if not self.enable_moveit_planning or self.move_group_client is None:
                    self._abort_sequence(sequence_name, step_name, "MoveIt not enabled")
                    return

                joint_msg = self._build_joint_state_message(effective_step)
                if self._joint_goal_already_reached(joint_msg):
                    self.get_logger().info(f'{step_name}: already at joint goal with settled feedback; no motion sent')
                    continue
                self.joint_target_pub.publish(joint_msg)

                context = self._plan_joint_state(step_name, joint_msg, effective_step)
                if context is None or not self._wait_for_plan(context, effective_step):
                    self._abort_sequence(sequence_name, step_name, "planning/execution failed")
                    return

                if self.wait_for_trajectory_ack:
                    self.get_logger().info("  Waiting for Arduino to finish trajectory...")
                    if not self._wait_for_arduino_done(
                        timeout_sec=60.0,
                        label=f"{sequence_name}/{step_name} trajectory",
                    ):
                        self._force_apply_pending_joint_state(self._arduino_error_message or "trajectory acknowledgement/endpoint verification failed")
                        if not bool(effective_step.get(
                            "continue_on_arduino_timeout",
                            self.continue_on_arduino_timeout,
                        )):
                            self._abort_sequence(sequence_name, step_name, self._arduino_error_message or "trajectory acknowledgement/endpoint verification failed")
                            return
                        self.get_logger().warn(f"  Continuing after trajectory timeout for {step_name}")
                else:
                    self.get_logger().info("  Continuing without trajectory DONE acknowledgement")
                    self._force_apply_pending_joint_state(
                        "trajectory acknowledgement disabled"
                    )
            else:
                self._abort_sequence(
                    sequence_name, step_name, f"unknown step type '{step_type}'"
                )
                return

        self._save_sequence_trajectory_csv(sequence_name)
        suffix = '; grasp retention requires verification (no gripper contact sensor)' if self._holding_cup else ''
        self._publish_status(f"Sequence {sequence_name} motion completed{suffix}")
        self.get_logger().info(f"=== Sequence '{sequence_name}' completed ===")

    def _abort_sequence(self, sequence_name: str, step_name: str, reason: str) -> None:
        self._save_sequence_trajectory_csv(sequence_name)
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
    def _extract_trajectory_waypoints(self, name: str, planned_trajectory, command_cfg: Optional[Dict] = None):
        """Extract MoveIt waypoints into raw joint values and Arduino strings."""
        command_cfg = command_cfg or {}
        jt = planned_trajectory.joint_trajectory
        validation_error = self._trajectory_validation_error(jt)
        if validation_error:
            self.get_logger().error(f"Unsafe trajectory for {name}: {validation_error}")
            return None

        joint_indices = {jn: idx for idx, jn in enumerate(jt.joint_names)}
        gripper_value = command_cfg.get("gripper_value")
        final_gripper_value = command_cfg.get("final_gripper_value")

        # Log raw MoveIt joint names and trajectory info
        self.get_logger().info(f"  MoveIt joints: {jt.joint_names}")
        self.get_logger().info(f"  Trajectory has {len(jt.points)} waypoints")

        # Joints that aren't part of the planning group (e.g. joint_grip_L for
        # the base_arm group) must hold their last commanded value, not 0.
        held = (
            dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
            if self.latest_joint_state is not None else {}
        )

        arduino_waypoints = []
        raw_waypoints = []
        previous_theta = held.get('base_theta', 0.0)
        previous_milliseconds = -1
        for point_index, point in enumerate(jt.points):
            values = []
            for joint_name in TRAJECTORY_JOINT_ORDER:
                idx = joint_indices.get(joint_name)
                if idx is not None:
                    values.append(point.positions[idx])
                else:
                    values.append(held.get(joint_name, 0.0))
            # YAML and Arduino use gripper degrees; RobotState/JointState must
            # always use SI radians like every other revolute ROS joint.
            if gripper_value is not None:
                values[TRAJECTORY_JOINT_ORDER.index("joint_grip_L")] = math.radians(
                    float(gripper_value)
                )
            if final_gripper_value is not None and point_index == len(jt.points) - 1:
                values[TRAJECTORY_JOINT_ORDER.index("joint_grip_L")] = math.radians(
                    float(final_gripper_value)
                )
            theta_index = TRAJECTORY_JOINT_ORDER.index('base_theta')
            values[theta_index] = previous_theta + math.atan2(math.sin(values[theta_index]-previous_theta),
                                                             math.cos(values[theta_index]-previous_theta))
            previous_theta = values[theta_index]
            raw_waypoints.append(list(values))
            milliseconds = max(previous_milliseconds + 1,
                               math.ceil(point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1e6))
            previous_milliseconds = milliseconds
            arduino_waypoints.append(self._format_waypoint_for_arduino(values) + f" {milliseconds}")

        from arduino_reader import validate_command
        try:
            for waypoint in arduino_waypoints:
                fields = waypoint.split()
                validate_command('CMD 1 ' + ' '.join(fields[:-1] + ['0']))
        except ValueError as exc:
            self.get_logger().error(f'Trajectory exceeds hardware limits: {exc}')
            return None

        # Log first and last waypoint (raw radians/meters and Arduino format)
        self.get_logger().info(f"  Raw first: {raw_waypoints[0]}")
        self.get_logger().info(f"  Raw last:  {raw_waypoints[-1]}")
        self.get_logger().info(f"  Arduino first: {arduino_waypoints[0]}")
        self.get_logger().info(f"  Arduino last:  {arduino_waypoints[-1]}")
        if self.pretty_print_trajectory_waypoints:
            self._log_trajectory_waypoints(name, jt, raw_waypoints, arduino_waypoints)

        return jt, raw_waypoints, arduino_waypoints

    def _trajectory_validation_error(self, jt) -> Optional[str]:
        """Return a reason when a MoveIt trajectory is unsafe to forward."""
        if not jt.points:
            return "trajectory contains no waypoints"
        if not jt.joint_names:
            return "trajectory contains no joints"
        if len(set(jt.joint_names)) != len(jt.joint_names):
            return "trajectory contains duplicate joint names"
        unknown = sorted(set(jt.joint_names) - set(TRAJECTORY_JOINT_ORDER))
        if unknown:
            return f"trajectory contains joints unsupported by Arduino: {unknown}"

        previous_time = -1.0
        for index, point in enumerate(jt.points):
            if len(point.positions) != len(jt.joint_names):
                return (
                    f"waypoint {index} has {len(point.positions)} positions for "
                    f"{len(jt.joint_names)} joints"
                )
            if not all(math.isfinite(value) for value in point.positions):
                return f"waypoint {index} contains a non-finite position"
            point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if point_time < previous_time:
                return f"waypoint {index} moves backward in time"
            previous_time = point_time

        if self.latest_joint_state is not None and self.max_start_state_error > 0.0:
            current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
            first = jt.points[0].positions
            for index, joint_name in enumerate(jt.joint_names):
                if joint_name not in current:
                    continue
                delta = float(first[index]) - float(current[joint_name])
                error = abs(math.atan2(math.sin(delta), math.cos(delta))) if joint_name == 'base_theta' else abs(delta)
                if error > self.max_start_state_error:
                    return (
                        f"first waypoint for {joint_name} is {error:.3f} from current "
                        f"state (limit {self.max_start_state_error:.3f})"
                    )
        return None

    def _record_planned_trajectory(
        self,
        name: str,
        jt,
        raw_waypoints: List[List[float]],
        arduino_waypoints: List[str],
    ) -> None:
        self._save_trajectory_csv(name, jt, raw_waypoints, arduino_waypoints)
        self._append_sequence_trajectory(jt, raw_waypoints, arduino_waypoints)

    def _send_trajectory_to_arduino(self, name, raw_waypoints, arduino_waypoints, endpoint_profile="precise"):
        from arduino_reader import validate_command
        self._require_fresh_state()
        actual = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        for j, joint in enumerate(TRAJECTORY_JOINT_ORDER):
            if joint == 'joint_grip_L':
                continue
            limit = 0.01 if joint in ('base_x', 'base_y') else 0.04
            if abs(actual[joint] - raw_waypoints[0][j]) > limit:
                raise RuntimeError(f'{name}: start moved for {joint}; replan required')
        self._arm_arduino_wait()
        if endpoint_profile not in ('precise', 'transit'):
            raise ValueError('unknown endpoint profile')
        self._endpoint_profile = endpoint_profile
        self._queue_pending_joint_state(raw_waypoints[-1], f'{name} trajectory')
        # One transaction and one DONE per subtask. The bridge feeds a bounded
        # firmware buffer without changing timestamps or stopping at packets.
        queue = ','.join(arduino_waypoints)
        _, duration = validate_command(f'CMD {self._command_id} {queue}', streaming=True)
        self._command_expected_duration = duration
        prefix = 'TRANSIT' if endpoint_profile == 'transit' else 'STREAM'
        self.arduino_cmd_pub.publish(String(data=f'{prefix} {self._command_id} {queue}'))
        self.get_logger().info(f'{name}: streaming {len(arduino_waypoints)} samples as one motion ({duration:.2f}s)')
        if self.dry_run:
            self._complete_dry_run_motion(name, raw_waypoints[-1])

    def _extract_and_send_trajectory(self, name: str, planned_trajectory, command_cfg: Optional[Dict] = None) -> bool:
        """Extract, save, and send a MoveIt planned trajectory to Arduino."""
        extracted = self._extract_trajectory_waypoints(name, planned_trajectory, command_cfg)
        if extracted is None:
            return False
        jt, raw_waypoints, arduino_waypoints = extracted
        self._record_planned_trajectory(name, jt, raw_waypoints, arduino_waypoints)
        self._send_trajectory_to_arduino(name, raw_waypoints, arduino_waypoints,
                    (command_cfg or {}).get("endpoint_profile", "precise"))
        return True

    def _log_trajectory_waypoints(
        self,
        name: str,
        jt,
        raw_waypoints: List[List[float]],
        arduino_waypoints: List[str],
    ) -> None:
        self.get_logger().info(
            f"  Full waypoint list for {name} "
            f"({len(raw_waypoints)} waypoint(s), Arduino order)"
        )
        self.get_logger().info(
            "  idx | t_sec | base_x_m base_y_m base_theta_rad grip "
            "hand_rad wrist_rad elbow_rad shoulder_rad | arduino"
        )
        for i, point in enumerate(jt.points):
            t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            raw = raw_waypoints[i]
            raw_text = " ".join(f"{v: .4f}" for v in raw)
            self.get_logger().info(
                f"  {i:03d} | {t:6.3f} | {raw_text} | {arduino_waypoints[i]}"
            )

    def _start_sequence_trajectory_log(self, sequence_name: str) -> None:
        self._sequence_log_name = sequence_name
        self._sequence_log_rows = []
        self._sequence_log_time_offset = 0.0
        self._start_target_audit(sequence_name)

    def _start_target_audit(self, sequence_name: str) -> None:
        """Create the per-sequence Cartesian-target audit before any motion."""
        import csv
        from datetime import datetime

        try:
            log_dir = Path.cwd() / "target_logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._target_audit_path = (
                log_dir / f"targets_{sequence_name}_{timestamp}.csv"
            )
            with self._target_audit_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow([
                    "recorded_at", "sequence", "step", "detection_topic",
                    "detection_stamp_sec", "detection_stamp_nanosec",
                    "detection_frame", "raw_detected_x", "raw_detected_y",
                    "raw_detected_z", "planning_frame",
                    "transformed_detected_x", "transformed_detected_y",
                    "transformed_detected_z", "offset_x", "offset_y", "offset_z",
                    "requested_target_z", "tcp_height_adjustment_m",
                    "target_x", "target_y", "target_z",
                    "tolerance_xy_m", "tolerance_z_m", "end_effector_link",
                ])
            self.get_logger().info(f"  Target audit log: {self._target_audit_path}")
        except Exception as exc:
            self._target_audit_path = None
            self.get_logger().warn(f"  Failed to create target audit log: {exc}")

    def _append_target_audit(
        self,
        sequence_name: str,
        step_name: str,
        detection_topic: str,
        detected_pose: PoseStamped,
        resolved_step: Dict,
    ) -> None:
        """Append the exact detected and resolved Cartesian target used by MoveIt."""
        import csv
        from datetime import datetime

        if self._target_audit_path is None:
            return

        raw = detected_pose.pose.position
        target = resolved_step["position"]
        requested_target_z = float(
            resolved_step.get("_unclamped_position_z_m", target[2])
        )
        tcp_height_adjustment = float(target[2]) - requested_target_z
        ox = float(resolved_step.get("offset_x", 0.0))
        oy = float(resolved_step.get("offset_y", 0.0))
        oz = float(resolved_step.get("offset_z", 0.0))
        transformed = [
            target[0] - ox,
            target[1] - oy,
            requested_target_z - oz,
        ]
        default_tol = float(
            resolved_step.get("position_tolerance_m", self.position_tolerance_m)
        )
        tolerance_xy = float(
            resolved_step.get("position_tolerance_xy_m", default_tol)
        )
        tolerance_z = float(
            resolved_step.get("position_tolerance_z_m", default_tol)
        )

        try:
            with self._target_audit_path.open("a", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow([
                    datetime.now().astimezone().isoformat(),
                    sequence_name,
                    step_name,
                    detection_topic,
                    detected_pose.header.stamp.sec,
                    detected_pose.header.stamp.nanosec,
                    detected_pose.header.frame_id,
                    f"{raw.x:.6f}", f"{raw.y:.6f}", f"{raw.z:.6f}",
                    resolved_step.get("frame_id", self.base_frame),
                    f"{transformed[0]:.6f}",
                    f"{transformed[1]:.6f}",
                    f"{transformed[2]:.6f}",
                    f"{ox:.6f}", f"{oy:.6f}", f"{oz:.6f}",
                    f"{requested_target_z:.6f}",
                    f"{tcp_height_adjustment:.6f}",
                    f"{target[0]:.6f}",
                    f"{target[1]:.6f}",
                    f"{target[2]:.6f}",
                    f"{tolerance_xy:.6f}",
                    f"{tolerance_z:.6f}",
                    resolved_step.get("link_name", self.end_effector_link),
                ])
        except Exception as exc:
            self.get_logger().warn(f"  Failed to append target audit row: {exc}")

    def _append_sequence_trajectory(self, jt, raw_waypoints, arduino_waypoints) -> None:
        if self._sequence_log_name is None:
            return

        start_index = 0
        if self._sequence_log_rows and raw_waypoints:
            previous = self._sequence_log_rows[-1]["raw"]
            if all(abs(a - b) < 1e-9 for a, b in zip(previous, raw_waypoints[0])):
                start_index = 1

        last_time = self._sequence_log_time_offset
        for i in range(start_index, len(raw_waypoints)):
            point = jt.points[i]
            segment_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            t = self._sequence_log_time_offset + segment_time
            self._sequence_log_rows.append({
                "time": t,
                "raw": list(raw_waypoints[i]),
                "arduino": arduino_waypoints[i].split(),
            })
            last_time = t

        if jt.points:
            segment_duration = (
                jt.points[-1].time_from_start.sec
                + jt.points[-1].time_from_start.nanosec / 1e9
            )
            self._sequence_log_time_offset += segment_duration
            if start_index < len(raw_waypoints):
                self._sequence_log_time_offset = max(self._sequence_log_time_offset, last_time)

    def _save_sequence_trajectory_csv(self, sequence_name: str) -> None:
        if self._sequence_log_name != sequence_name or not self._sequence_log_rows:
            self._sequence_log_name = None
            self._sequence_log_rows = []
            self._sequence_log_time_offset = 0.0
            return

        try:
            from datetime import datetime

            log_dir = Path.cwd() / "trajectory_logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = log_dir / f"traj_{sequence_name}_combined_{timestamp}.csv"
            self._write_trajectory_rows_csv(csv_path, self._sequence_log_rows)
            self.get_logger().info(
                f"  Combined sequence trajectory saved: {csv_path} "
                f"({len(self._sequence_log_rows)} waypoints)"
            )
        except Exception as exc:
            self.get_logger().warn(f"  Failed to save combined sequence trajectory CSV: {exc}")
        finally:
            self._sequence_log_name = None
            self._sequence_log_rows = []
            self._sequence_log_time_offset = 0.0

    def _save_trajectory_csv(self, name: str, jt, raw_waypoints, arduino_waypoints) -> None:
        """Save planned trajectory to CSV for debugging."""
        from datetime import datetime

        try:
            log_dir = Path.cwd() / "trajectory_logs"
            log_dir.mkdir(parents=True, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_name = "".join(c if c.isalnum() or c in "_-" else "_" for c in name)
            csv_path = log_dir / f"traj_{safe_name}_{timestamp}.csv"

            rows = []
            for i, point in enumerate(jt.points):
                t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                rows.append({
                    "time": t,
                    "raw": list(raw_waypoints[i]),
                    "arduino": arduino_waypoints[i].split(),
                })
            self._write_trajectory_rows_csv(csv_path, rows)

            self.get_logger().info(f"  Trajectory saved: {csv_path}")
        except Exception as e:
            self.get_logger().warn(f"  Failed to save trajectory CSV: {e}")

    @staticmethod
    def _write_trajectory_rows_csv(csv_path: Path, rows: List[Dict]) -> None:
        import csv

        raw_header = [f"raw_{j}" for j in TRAJECTORY_JOINT_ORDER]
        arduino_header = [
            "arduino_x_cm", "arduino_y_cm", "arduino_theta_deg",
            "arduino_grip", "arduino_hand_deg", "arduino_wrist_deg",
            "arduino_elbow_deg", "arduino_shoulder_deg",
            "arduino_la", "arduino_frame",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["waypoint", "time_from_start"] + raw_header + arduino_header)
            for i, row in enumerate(rows):
                writer.writerow(
                    [i, f"{float(row['time']):.3f}"]
                    + [f"{float(v):.6f}" for v in row["raw"]]
                    + list(row["arduino"])
                )

    @staticmethod
    def _format_waypoint_for_arduino(values: List[float]) -> str:
        """Convert joint values to Arduino format: m→cm, rad→deg."""
        if len(values) < 8:
            return "0 0 0 0 0 0 0 0"

        base_x_cm = values[0] * 100.0         # m → cm
        base_y_cm = values[1] * 100.0         # m → cm
        theta_deg = math.degrees(values[2])   # rad → deg
        grip = math.degrees(values[3])        # rad → Arduino degrees
        hand_deg = math.degrees(values[4])    # rad → deg
        wrist_deg = math.degrees(values[5])   # rad → deg
        elbow_deg = math.degrees(values[6])   # rad → deg
        shoulder_deg = math.degrees(values[7])  # rad → deg
        la = 0.0                               # linear actuator (unused)
        frame = 0.0                            # frame (unused)

        return (
            f"{base_x_cm:.3f} {base_y_cm:.3f} {theta_deg:.3f} "
            f"{grip:.3f} {hand_deg:.3f} {wrist_deg:.3f} "
            f"{elbow_deg:.3f} {shoulder_deg:.3f} {la:.3f} {frame:.3f}"
        )

    def _send_gripper_to_arduino(self, grip_value: float) -> None:
        """Send a full 10-value waypoint to Arduino with current joint positions and updated grip."""
        if self.latest_joint_state is None:
            self.get_logger().warn("No joint state available — cannot send gripper command")
            return

        current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        values = []
        for joint_name in TRAJECTORY_JOINT_ORDER:
            if joint_name == "joint_grip_L":
                values.append(math.radians(grip_value))
            else:
                values.append(current.get(joint_name, 0.0))

        self._require_fresh_state()
        waypoint = self._format_waypoint_for_arduino(values) + " 0"
        # Arm the DONE event BEFORE publishing — see _wait_for_arduino_done.
        self._arm_arduino_wait()
        self._queue_pending_joint_state(values, "gripper command")
        self.arduino_cmd_pub.publish(String(data=f"CMD {self._command_id} {waypoint}"))
        self.get_logger().info(f"Sent gripper waypoint to Arduino: {waypoint}")
        if self.dry_run:
            self._complete_dry_run_motion("gripper", values)

    def _complete_dry_run_motion(self, name: str, final_values: List[float]) -> None:
        """Pretend the hardware reached the commanded endpoint instantly."""
        self._virtual_positions = dict(zip(TRAJECTORY_JOINT_ORDER, final_values))
        self.get_logger().warn(
            f"DRY RUN: {name} treated as executed; virtual state "
            + ", ".join(f"{n}={v:.3f}" for n, v in self._virtual_positions.items()))
        self._arduino_done_event.set()

    # ------------------------------------------------------------------
    # Gripper value note: 0 = fully open, 90 = closed
    # Arduino expects 10 space-separated values per waypoint:
    #   x_cm y_cm theta_deg grip hand wrist elbow shoulder la frame
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # MoveIt planning
    # ------------------------------------------------------------------
    def _joint_state_with_updates(self, updates: Dict[str, float]) -> JointState:
        """Return the current full JointState with selected positions replaced."""
        if self.latest_joint_state is not None:
            joint_state = deepcopy(self.latest_joint_state)
        else:
            joint_state = JointState()
        values = dict(zip(joint_state.name, joint_state.position))
        values.update({name: float(value) for name, value in updates.items()})
        ordered_names = list(joint_state.name)
        for name in TRAJECTORY_JOINT_ORDER:
            if name not in ordered_names:
                ordered_names.append(name)
        joint_state.name = ordered_names
        joint_state.position = [float(values.get(name, 0.0)) for name in ordered_names]
        joint_state.velocity = []
        joint_state.effort = []
        joint_state.header.stamp = self.get_clock().now().to_msg()
        return joint_state

    def _plan_joint_phase(
        self,
        name: str,
        group_name: str,
        start_state: JointState,
        targets: Dict[str, float],
        command_cfg: Dict,
    ) -> Optional[RobotTrajectory]:
        """Plan one collision-checked joint-space phase through MoveIt."""
        service_timeout = max(
            1.0,
            float(command_cfg.get("planning_service_timeout_sec", 3.0)),
        )
        if not self.motion_plan_client.wait_for_service(
            timeout_sec=service_timeout
        ):
            self.get_logger().error(
                f"  Cannot plan {name}: {self.planning_service_name} is unavailable"
            )
            return None

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = group_name
        motion_request.planner_id = str(
            command_cfg.get("planner_id", self.planner_id)
        )
        motion_request.allowed_planning_time = float(
            command_cfg.get("phase_allowed_planning_time", 5.0)
        )
        motion_request.num_planning_attempts = int(
            command_cfg.get("phase_num_planning_attempts", 3)
        )
        motion_request.start_state = RobotState()
        motion_request.start_state.is_diff = True
        motion_request.start_state.joint_state = deepcopy(start_state)

        if not self._state_valid(start_state, group_name, log=True):
            return None
        goal_state = deepcopy(start_state)
        goal_state.position = [targets.get(n, p) for n, p in zip(goal_state.name, goal_state.position)]
        if not self._state_valid(goal_state, group_name, log=True):
            return None
        motion_request.max_velocity_scaling_factor = float(command_cfg.get('velocity_scaling', 1.0))
        motion_request.max_acceleration_scaling_factor = float(command_cfg.get('acceleration_scaling', 1.0))
        goal_constraints = Constraints()
        goal_constraints.name = name
        tolerance = max(
            1e-4,
            float(command_cfg.get("phase_joint_tolerance", 0.002)),
        )
        for joint_name, position in targets.items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = float(position)
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance
            constraint.weight = 1.0
            goal_constraints.joint_constraints.append(constraint)
        motion_request.goal_constraints = [goal_constraints]
        if '_path_constraints' in command_cfg:
            motion_request.path_constraints = deepcopy(command_cfg['_path_constraints'])
        if group_name == 'base':
            start = dict(zip(start_state.name, start_state.position))
            for joint, target in targets.items():
                lo, hi = sorted((start[joint], target))
                margin = 0.10 if joint != 'base_theta' else 0.15
                motion_request.path_constraints.joint_constraints.append(JointConstraint(
                    joint_name=joint, position=(lo+hi)/2,
                    tolerance_above=(hi-lo)/2+margin, tolerance_below=(hi-lo)/2+margin, weight=1.0))
        self._fill_workspace(motion_request, self.base_frame)

        self.get_logger().info(
            f"  Planning collision-checked {name} phase with group '{group_name}'"
        )
        future = self.motion_plan_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        wait_timeout = motion_request.allowed_planning_time + service_timeout + 2.0
        if not completed.wait(timeout=wait_timeout):
            self.get_logger().error(
                f"  {name} phase planning timed out after {wait_timeout:.1f}s"
            )
            return None
        try:
            response = future.result().motion_plan_response
        except Exception as exc:
            self.get_logger().error(f"  {name} phase planning failed: {exc}")
            return None

        error_code = int(response.error_code.val)
        if error_code != 1:
            error_name = MOVEIT_ERROR_NAMES.get(error_code, str(error_code))
            self.get_logger().error(
                f"  {name} phase has no valid collision-free path: {error_name}"
            )
            return None
        if not response.trajectory.joint_trajectory.points:
            self.get_logger().error(f"  {name} phase returned an empty trajectory")
            return None
        return response.trajectory

    @staticmethod
    def _trajectory_final_positions(trajectory: RobotTrajectory) -> Dict[str, float]:
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            return {}
        return dict(
            zip(joint_trajectory.joint_names, joint_trajectory.points[-1].positions)
        )

    def _clear_and_rebuild_octomap(self, name: str, command_cfg: Dict) -> bool:
        enabled = bool(
            command_cfg.get(
                "clear_octomap_before_plan",
                self.clear_octomap_before_pose_plan,
            )
        )
        if not enabled or not self.clear_octomap_before_pose_plan:
            return True

        timeout_sec = float(
            command_cfg.get(
                "octomap_clear_timeout_sec", self.octomap_clear_timeout_sec
            )
        )
        rebuild_wait_sec = float(
            command_cfg.get(
                "octomap_rebuild_wait_sec", self.octomap_rebuild_wait_sec
            )
        )
        if not self.octomap_clear_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"  Cannot prepare OctoMap for {name}: clear service unavailable"
            )
            return False

        self.get_logger().info(
            f"  Clearing OctoMap before planning {name}"
        )
        requested_stamp = self.get_clock().now().nanoseconds
        future = self.octomap_clear_client.call_async(Empty.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _done: completed.set())
        if not completed.wait(timeout=timeout_sec):
            self.get_logger().error(
                f"  Cannot prepare OctoMap for {name}: clear timed out after "
                f"{timeout_sec:.1f}s"
            )
            return False
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(
                f"  Cannot prepare OctoMap for {name}: clear failed: {exc}"
            )
            return False

        deadline = time.monotonic() + max(3.0, rebuild_wait_sec)
        while time.monotonic() < deadline:
            if self._cloud_stamp_ns > requested_stamp:
                # Allow MoveIt's subscriber to insert the corresponding cloud.
                time.sleep(0.2)
                return True
            time.sleep(0.05)
        self.get_logger().error(f'{name}: no fresh target-masked cloud after map clear')
        return False

    def _add_local_base_goal_constraints(self, constraints, command_cfg, name, pose_msg):
        if self.latest_joint_state is None:
            raise RuntimeError('Base goal bounds require measured state')
        current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        def bounded(joint, low, high):
            if not math.isfinite(low + high) or low > high:
                raise ValueError(f'Invalid bounds for {joint}')
            constraints.joint_constraints.append(JointConstraint(joint_name=joint,
                position=(low+high)/2, tolerance_above=(high-low)/2,
                tolerance_below=(high-low)/2, weight=1.0))
        minimum = command_cfg.get('base_goal_tcp_x_min_m')
        maximum = command_cfg.get('base_goal_tcp_x_max_m')
        if minimum is not None and maximum is not None:
            q = pose_msg.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            c, sn = math.cos(yaw), math.sin(yaw)
            lateral = float(command_cfg.get('base_goal_tcp_y_max_m', 0.15))
            corners = [(pose_msg.pose.position.x - c*f + sn*l,
                        pose_msg.pose.position.y - sn*f - c*l)
                       for f in (float(minimum), float(maximum)) for l in (-lateral,lateral)]
            bounded('base_x', min(p[0] for p in corners), max(p[0] for p in corners))
            bounded('base_y', min(p[1] for p in corners), max(p[1] for p in corners))
            yaw = current['base_theta'] + math.atan2(math.sin(yaw-current['base_theta']), math.cos(yaw-current['base_theta']))
            limit = float(command_cfg.get('max_base_goal_rotation_rad', 0.15))
            bounded('base_theta', yaw-limit, yaw+limit)
        else:
            for joint, key in (('base_x','max_base_goal_translation_m'),
                               ('base_y','max_base_goal_y_translation_m'),
                               ('base_theta','max_base_goal_rotation_rad')):
                limit = command_cfg.get(key, command_cfg.get('max_base_goal_translation_m') if joint=='base_y' else None)
                if limit is not None:
                    bounded(joint, current[joint]-float(limit), current[joint]+float(limit))

    def _geometry_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('task_id') == 1:
                self._cup_geometry = data
        except (ValueError, TypeError):
            pass

    def _update_cup_scene(self, pose):
        stamp = Time.from_msg(pose.header.stamp).nanoseconds
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            data = self._cup_geometry
            if data is not None and abs(int(data['stamp_ns']) - stamp) <= 2_000_000_000:
                break
            time.sleep(0.02)
        data = self._cup_geometry
        if data is None or abs(int(data['stamp_ns']) - stamp) > 2_000_000_000:
            raise RuntimeError('No fresh cup dimensions for collision planning')
        obj = CollisionObject()
        obj.id = 'detected_cup'
        obj.header.frame_id = self.base_frame
        obj.operation = CollisionObject.ADD
        cylinder = SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                  dimensions=[float(data['height']), float(data['radius'])])
        center = Pose()
        center.position = deepcopy(pose.pose.position)
        center.position.z -= 0.15 * float(data['height'])
        if data.get('source') == 'configured_collision_envelope':
            # Use the stable sequence observation, not a newer jittered
            # detection, while retaining the envelope's surface-relative offset.
            observation = data.get('center')
            envelope = data.get('collision_center')
            if observation is None or envelope is None:
                raise RuntimeError('Cup envelope lacks surface-relative center; restart vision')
            delta = [float(b)-float(a) for a, b in zip(observation, envelope)]
            if len(delta) != 3 or not all(math.isfinite(v) for v in delta):
                raise RuntimeError('Invalid cup collision center')
            center.position.x = pose.pose.position.x + delta[0]
            center.position.y = pose.pose.position.y + delta[1]
            center.position.z = pose.pose.position.z + delta[2]
        center.orientation.w = 1.0
        obj.primitives = [cylinder]
        obj.primitive_poses = [center]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obj]
        # Preserve all existing ACM entries; allow only intentional finger contact,
        # and contact with the detected table box (homeplus_vision table_plane)
        # the cup stands on: otherwise the held cup makes the lift's start invalid.
        get = GetPlanningScene.Request()
        get.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self._call_service(self.scene_client, get).scene.allowed_collision_matrix
        names = ['detected_cup', 'FingerL_new_1', 'FingerR_new_1', 'detected_table']
        for name in names:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        cup = acm.entry_names.index('detected_cup')
        for other in names[1:]:
            i = acm.entry_names.index(other)
            acm.entry_values[cup].enabled[i] = True
            acm.entry_values[i].enabled[cup] = True
        request.scene.allowed_collision_matrix = acm
        if not self._call_service(self.apply_scene_client, request).success:
            raise RuntimeError('Could not add detected cup collision geometry')
        self._cup_collision_object = obj

    def _set_allowed_collisions(self, pairs, allowed: bool) -> None:
        """Set ACM entries for (a, b) name pairs, preserving every other entry."""
        get = GetPlanningScene.Request()
        get.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self._call_service(self.scene_client, get).scene.allowed_collision_matrix
        for name in {n for pair in pairs for n in pair}:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        for a, b in pairs:
            i, j = acm.entry_names.index(a), acm.entry_names.index(b)
            acm.entry_values[i].enabled[j] = allowed
            acm.entry_values[j].enabled[i] = allowed
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.allowed_collision_matrix = acm
        if not self._call_service(self.apply_scene_client, request).success:
            raise RuntimeError('Could not update the allowed collision matrix')

    def _detach_cup(self):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = 'grasp_tcp'
        attached.object.id = 'detected_cup'
        attached.object.operation = CollisionObject.REMOVE
        req.scene.robot_state.attached_collision_objects = [attached]
        if not self._call_service(self.apply_scene_client, req).success:
            raise RuntimeError('Could not detach cup collision geometry')
        self._set_allowed_collisions(self._FINGER_OCTOMAP_PAIRS, False)
        self._holding_cup = False
        self._cup_collision_object = None

    _FINGER_OCTOMAP_PAIRS = [('<octomap>', 'FingerL_new_1'), ('<octomap>', 'FingerR_new_1')]

    def _attach_cup(self):
        if self._cup_collision_object is None:
            raise RuntimeError('Missing cup geometry at closure')
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = 'grasp_tcp'
        attached.touch_links = ['FingerL_new_1', 'FingerR_new_1']
        attached.object = deepcopy(self._cup_collision_object)
        req.scene.robot_state.attached_collision_objects = [attached]
        if not self._call_service(self.apply_scene_client, req).success:
            raise RuntimeError('Could not attach cup collision geometry')
        self._holding_cup = True
        # Voxels at the closed fingers are the held cup's own surface (the mask
        # never removes all of it), so let the fingers touch the OctoMap until
        # release. Other links, and the attached cup body, are still checked.
        self._set_allowed_collisions(self._FINGER_OCTOMAP_PAIRS, True)
        # Removing a world object that was never added makes MoveIt reject the
        # whole diff, so clear any stale world cup in a separate best-effort call.
        remove_req = ApplyPlanningScene.Request()
        remove_req.scene.is_diff = True
        remove_req.scene.robot_state.is_diff = True
        remove_req.scene.world.collision_objects = [
            CollisionObject(id='detected_cup', operation=CollisionObject.REMOVE)]
        self._call_service(self.apply_scene_client, remove_req)

    def _call_service(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'MoveIt service unavailable: {client.srv_name}')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            future.cancel()
            raise RuntimeError(f'MoveIt service timeout: {client.srv_name}')
        return future.result()

    def _state_valid(self, joints, group, constraints=None, log=False):
        req = GetStateValidity.Request()
        req.robot_state.joint_state = deepcopy(joints)
        req.robot_state.is_diff = True
        req.group_name = group
        if constraints is not None:
            req.constraints = constraints
        response = self._call_service(self.validity_client, req)
        if not response.valid and log:
            contacts = sorted({f'{c.contact_body_1}/{c.contact_body_2}' for c in response.contacts})
            self.get_logger().error(f'Invalid state for {group}: contacts={contacts}, '
                                   f'constraint results={response.constraint_result}')
            self._publish_collision_debug(req.robot_state, response.contacts)
        return response.valid

    def _republish_collision_debug(self):
        snapshot = getattr(self, '_collision_debug', None)
        if snapshot is not None:
            self.collision_state_pub.publish(snapshot[0])
            self.collision_markers_pub.publish(snapshot[1])

    def _clear_collision_debug(self):
        if not hasattr(self, 'collision_state_pub'):
            return
        self._collision_debug = (DisplayRobotState(hide=True),
                                 MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self._republish_collision_debug()

    def _publish_collision_debug(self, robot_state, contacts):
        """Visualization only: never publish on joint_states or execution topics."""
        if not hasattr(self, 'collision_state_pub'):
            return
        display = DisplayRobotState(state=deepcopy(robot_state), hide=False)
        display.state.is_diff = False
        markers = [Marker(action=Marker.DELETEALL)]
        highlighted = set()
        for i, contact in enumerate(contacts):
            frame = contact.header.frame_id or self.base_frame
            pair = f'{contact.contact_body_1} / {contact.contact_body_2}'
            for body, body_type in ((contact.contact_body_1, contact.body_type_1),
                                    (contact.contact_body_2, contact.body_type_2)):
                if body_type == contact.ROBOT_LINK and body not in highlighted:
                    display.highlight_links.append(ObjectColor(
                        id=body, color=ColorRGBA(r=1.0, g=0.15, b=0.05, a=0.85)))
                    highlighted.add(body)
            dot = Marker(ns='collision_points', id=i, type=Marker.SPHERE, action=Marker.ADD)
            dot.header.frame_id = frame
            # Zero stamp shows the fixed world-space diagnostic even later.
            dot.pose.position = deepcopy(contact.position)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.012
            dot.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            markers.append(dot)
            label = deepcopy(dot)
            label.ns = 'collision_labels'
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.z += 0.025 + i * 0.018
            label.scale.z = 0.018
            label.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0)
            label.text = f'REJECTED: {pair} ({contact.depth * 1000:.1f} mm overlap)'
            markers.append(label)
            self.get_logger().warning(
                f'Collision overlay: {pair} at {frame} '
                f'[{contact.position.x:.3f}, {contact.position.y:.3f}, {contact.position.z:.3f}]')
        self._collision_debug = (display, MarkerArray(markers=markers))
        self._republish_collision_debug()

    def _fk_pose(self, joints, link, frame=None):
        req = GetPositionFK.Request()
        req.header.frame_id = frame or self.base_frame
        req.fk_link_names = [link]
        req.robot_state.joint_state = deepcopy(joints)
        req.robot_state.is_diff = True
        response = self._call_service(self.fk_client, req)
        return response.pose_stamped[0] if response.error_code.val == 1 and response.pose_stamped else None

    @staticmethod
    def _group_joints(group):
        if group in ('base_arm', 'base_press_arm'):
            return BASE_JOINT_NAMES + ARM_JOINT_NAMES
        return BASE_JOINT_NAMES if group == 'base' else ARM_JOINT_NAMES

    def _joint_goal(self, joints, group):
        goal = Constraints()
        q = dict(zip(joints.name, joints.position))
        for name in self._group_joints(group):
            goal.joint_constraints.append(JointConstraint(joint_name=name, position=q[name],
                tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0))
        return goal

    # ------------------------------------------------------------------
    # Deliberate base placement. The arm's pitch joints move the TCP in one
    # vertical plane beside base_link; it cannot shift the gripper sideways
    # or yaw it, so a level grasp is fixed by where the base stands.
    # ------------------------------------------------------------------
    def _robot_description_callback(self, msg: String) -> None:
        self._robot_description = msg.data
        self._planar_arms = {}

    def _arm_model(self, link: Optional[str] = None) -> Optional[PlanarArm]:
        link = link or self.end_effector_link
        if link not in self._planar_arms:
            if self._robot_description is None:
                self.get_logger().error('Arm reach model needs the URDF; nothing received on /robot_description')
                return None
            try:
                arm = PlanarArm.from_urdf(self._robot_description, 'base_link', link)
                self._base_link_height = fixed_link_height(self._robot_description, 'base_link')
            except Exception as exc:
                self.get_logger().error(f'Arm reach model for {link} unavailable: {exc}')
                return None
            self._planar_arms[link] = arm
            self.get_logger().info(
                f'Arm reach model for {link}: pitch joints '
                f'{[j.name for j in arm.pitch_joints]}, roll held at 0 '
                f'{[j.name for j in arm.roll_joints]}, arm plane {arm.lateral_offset:+.4f} m '
                f'left of base_link')
        return self._planar_arms[link]

    def _measured_base(self):
        q = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        return q['base_x'], q['base_y'], q['base_theta']

    def _arm_plane_offset(self, target: PoseStamped, link: Optional[str] = None):
        """(sideways error of the target from the arm plane, forward distance)
        at the measured base pose."""
        arm = self._arm_model(link)
        if arm is None:
            return None
        self._require_fresh_state()
        bx, by, heading = self._measured_base()
        p = target.pose.position
        return arm_plane_error(bx, by, heading, (p.x, p.y), arm.lateral_offset)

    def _arm_only_reachable(self, steps, index, target: PoseStamped, forward: float) -> bool:
        """Whether the fixed-base pose steps after `index` can reach their
        waypoints level, from the measured base, for this target."""
        later = next((k for k in range(index + 1, len(steps))
                      if steps[k].get('type') == 'pose'), None)
        if later is None or steps[later].get('group_name') not in ('arm', 'press_arm'):
            return True
        arm = self._arm_model(steps[later].get('link_name'))
        offsets = self._arm_reach_offsets(steps, later)
        height = target.pose.position.z - self._base_link_height
        points = [(forward + dx, height + dz) for dx, dz in offsets]
        return any(arm.solve_path(points, branch, 0.0) is not None for branch in (1, -1))

    @staticmethod
    def _yaw_orientation(yaw: float) -> List[float]:
        return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]

    def _resolve_in_arm_plane(self, step: Dict, detected: PoseStamped) -> Optional[Dict]:
        """Fixed-base target: gripper heading = measured base heading, and the
        target moved sideways into the arm plane. The sideways residual is
        bounded by the refine step's max_arm_plane_error_m check."""
        link = step.get('link_name', self.end_effector_link)
        arm = self._arm_model(link)
        if arm is None:
            return None
        self._require_fresh_state()
        bx, by, heading = self._measured_base()
        p = detected.pose.position
        error, forward = arm_plane_error(bx, by, heading, (p.x, p.y), arm.lateral_offset)
        ox = float(step.get('offset_x', 0.0))
        oz = float(step.get('offset_z', 0.0))
        c, sn = math.cos(heading), math.sin(heading)
        resolved = dict(step)
        resolved['position'] = [p.x + sn * error + c * ox, p.y - c * error + sn * ox, p.z + oz]
        resolved['orientation'] = self._yaw_orientation(heading)
        resolved.pop('rpy_deg', None)
        resolved['frame_id'] = self.base_frame
        height = p.z + oz - self._base_link_height
        solutions = arm.solve_level(forward + ox, height)
        self.get_logger().info(
            f'  In-plane target at measured base ({bx:.3f}, {by:.3f}, {heading:.3f}): '
            f'{forward + ox:.3f} m ahead, height {height:.3f} m, target was '
            f'{error * 1000:+.1f} mm off the arm plane; {len(solutions)} arm solution(s)')
        if not solutions:
            return None
        current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        best = min(solutions, key=lambda q: sum((q[n] - current[n]) ** 2 for n in ARM_JOINT_NAMES))
        resolved['_preferred_arm_state'] = self._joint_state_with_updates(
            {n: best[n] for n in ARM_JOINT_NAMES})
        return resolved

    def _arm_reach_offsets(self, steps: List[Dict], index: int):
        """(forward, up) TCP offsets from the target, in the approach frame,
        for this step and the fixed-base arm steps that follow it, in order."""
        step = steps[index]
        link = step.get('link_name', self.end_effector_link)
        previous = (float(step.get('offset_x', 0.0)), float(step.get('offset_z', 0.0)))
        offsets = [previous]
        for later in steps[index + 1:]:
            if later.get('type') != 'pose':
                continue
            if later.get('group_name') not in ('arm', 'press_arm') or later.get('link_name', self.end_effector_link) != link:
                break
            if abs(float(later.get('offset_y', 0.0))) > 1e-9:
                raise ValueError(f"{later.get('name')}: the arm cannot reach sideways offsets")
            dx, dz = float(later.get('offset_x', 0.0)), float(later.get('offset_z', 0.0))
            if later.get('target') == 'detected':
                previous = (dx, dz)
            elif later.get('target') == 'previous_pose':
                previous = (previous[0] + dx, previous[1] + dz)
            else:
                break
            offsets.append(previous)
        return offsets

    def _plan_with_base_placement(self, name, step, detected_pose, steps, index):
        """Place the base from geometry, then plan base and arm motions.

        Heading points the arm plane at the target from the current base
        position; the reach is the centre of the forward interval in which the
        arm alone reaches every later fixed-base waypoint level. MoveIt checks
        each candidate for collisions and limits before anything moves.
        Returns the planned pose on success, otherwise None.
        """
        link = step.get('link_name', self.end_effector_link)
        arm = self._arm_model(link)
        if arm is None:
            return None
        settings = step['base_placement']
        try:
            offsets = self._arm_reach_offsets(steps, index)
        except ValueError as exc:
            self.get_logger().error(f'{name}: {exc}')
            return None
        self._require_fresh_state()
        bx, by, theta = self._measured_base()
        p = detected_pose.pose.position
        placements = plan_base_placements(
            arm, (p.x, p.y, p.z), (bx, by, theta), offsets,
            heading_offsets=[math.radians(float(v)) for v in settings.get('heading_offsets_deg', [0.0])],
            min_margin=float(settings.get('min_reach_margin_m', 0.03)),
            limit_margin=float(settings.get('joint_limit_margin_rad', 0.05)),
            base_height=self._base_link_height,
            preference=str(settings.get('reach_preference', 'centre')))
        self.get_logger().info(
            f'{name}: base placement for target ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) from base '
            f'({bx:.3f}, {by:.3f}, {theta:.3f}); arm waypoints (forward, up) {offsets}; '
            f'{len(placements)} candidate(s)')
        if not placements:
            self.get_logger().error(
                f'{name}: the arm alone cannot reach the waypoints level at this height with '
                f'{float(settings.get("min_reach_margin_m", 0.03)):.3f} m margin; no motion sent')
            return None
        groups: Dict[float, list] = {}
        for placement in placements:
            groups.setdefault(round(placement.heading, 6), []).append(placement)
        state = {'base_moved': False}
        ox = float(step.get('offset_x', 0.0))
        oz = float(step.get('offset_z', 0.0))
        for attempt, group in enumerate(groups.values()):
            heading = theta + wrap(group[0].heading - theta)
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = p.x + math.cos(heading) * ox
            pose.pose.position.y = p.y + math.sin(heading) * ox
            pose.pose.position.z = p.z + oz
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = self._yaw_orientation(heading)
            for placement in group:
                self.get_logger().info(
                    f'  candidate heading {heading:.3f} rad ({math.degrees(placement.heading_offset):+.0f} deg): '
                    f'base ({placement.base_x:.3f}, {placement.base_y:.3f}), target {placement.forward:.3f} m '
                    f'ahead, reach margin {placement.margin:.3f} m, arm '
                    + ', '.join(f'{n}={placement.arm[n]:.3f}' for n in ARM_JOINT_NAMES))
            cfg = dict(step)
            cfg['position'] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            cfg['orientation'] = self._yaw_orientation(heading)
            cfg['_placement_candidates'] = [
                self._joint_state_with_updates({'base_x': pl.base_x, 'base_y': pl.base_y,
                                                'base_theta': heading,
                                                **{n: pl.arm[n] for n in ARM_JOINT_NAMES}})
                for pl in group]
            cfg['_placement_target'] = deepcopy(detected_pose)
            cfg['_placement_state'] = state
            if attempt:
                cfg['clear_octomap_before_plan'] = False
            self.target_pose_pub.publish(pose)
            context = self._plan_pose(name, pose, cfg)
            if context is not None and self._wait_for_plan(context, cfg):
                return pose
            if state['base_moved']:
                # Never wander to another heading after the base has moved.
                self.get_logger().error(f'{name}: arm planning failed after base placement')
                return None
        return None

    def _validated_placements(self, pose, cfg, constraints):
        """Keep geometric placements that MoveIt confirms: FK agrees with the
        reach model, and the state is collision-free and meets the goal."""
        group = cfg.get('group_name', self.default_group_name)
        link = cfg.get('link_name', self.end_effector_link)
        accepted = []
        rejected = {}
        for candidate in cfg['_placement_candidates']:
            fk = self._fk_pose(candidate, link, pose.header.frame_id)
            if fk is None:
                rejected['fk_unavailable'] = rejected.get('fk_unavailable', 0) + 1
                continue
            mismatch = math.dist((fk.pose.position.x, fk.pose.position.y, fk.pose.position.z),
                                 (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
            if mismatch > 0.002:
                self.get_logger().error(f'  reach model and MoveIt FK disagree by {mismatch * 1000:.1f} mm')
                rejected['model_mismatch'] = rejected.get('model_mismatch', 0) + 1
                continue
            if not self._state_valid(candidate, group, constraints, log=True):
                rejected['collision_or_constraint'] = rejected.get('collision_or_constraint', 0) + 1
                continue
            accepted.append(candidate)
        self.get_logger().info(f'Placement check {group}: {len(accepted)} valid, rejected={rejected}')
        return accepted

    def _ik_candidates(self, pose, cfg, constraints):
        group = cfg.get('group_name', self.default_group_name)
        names = self._group_joints(group)
        start = deepcopy(self.latest_joint_state)
        current = dict(zip(start.name, start.position))
        preferred = cfg.get('_preferred_arm_state')
        preferred_values = dict(zip(preferred.name, preferred.position)) if preferred is not None else {}
        if preferred_values and group in ('arm', 'press_arm'):
            anchored = deepcopy(start)
            anchored.position = [preferred_values.get(n, p) if n in ARM_JOINT_NAMES else p
                                 for n, p in zip(anchored.name, anchored.position)]
            # Reuse only if it still satisfies the actual goal tolerances and
            # collisions at the MEASURED base pose. Never copy planned base joints.
            if self._state_valid(anchored, group, constraints):
                self.get_logger().info(f'IK preflight {group}: validated previous arm solution at measured base')
                return [anchored]
        rng = random.Random(0)  # Reproducible seed candidates; OMPL itself remains stochastic.
        accepted = []
        rejected = {}
        last_request = None
        for index in range(int(cfg.get('ik_candidate_attempts', 12))):
            seed = deepcopy(start)
            q = dict(current)
            if preferred_values and group in ('arm', 'press_arm'):
                q.update({n: preferred_values[n] for n in ARM_JOINT_NAMES if n in preferred_values})
            if index:
                for name in ARM_JOINT_NAMES:
                    q[name] += rng.uniform(-0.3, 0.3)
                for c in constraints.joint_constraints:
                    if c.joint_name in BASE_JOINT_NAMES:
                        q[c.joint_name] = c.position + rng.uniform(-c.tolerance_below, c.tolerance_above)
                bounds = {'joint_shoulder': (-0.523598, 2.617993),
                          'joint_elbow': (-0.610864, 2.443460),
                          'joint_wrist': (-1.570795, 1.570795),
                          'joint_hand': (-1.221729, 1.396262)}
                for name, (lo, hi) in bounds.items():
                    q[name] = max(lo, min(hi, q[name]))
            seed.position = [q[name] for name in seed.name]
            req = GetPositionIK.Request()
            req.ik_request.group_name = group
            req.ik_request.ik_link_name = cfg.get('link_name', self.end_effector_link)
            req.ik_request.pose_stamped = pose
            req.ik_request.robot_state.joint_state = seed
            req.ik_request.robot_state.is_diff = True
            req.ik_request.constraints = constraints
            req.ik_request.avoid_collisions = True
            req.ik_request.timeout = Duration(seconds=0.15).to_msg()
            last_request = req
            result = self._call_service(self.ik_client, req)
            if result.error_code.val != 1:
                key = MOVEIT_ERROR_NAMES.get(result.error_code.val, str(result.error_code.val))
                rejected[key] = rejected.get(key, 0) + 1
                continue
            candidate = result.solution.joint_state
            if not self._state_valid(candidate, group, constraints):
                rejected['constraint_or_collision'] = rejected.get('constraint_or_collision', 0) + 1
                continue
            values = dict(zip(candidate.name, candidate.position))
            if group in ('base_arm', 'base_press_arm'):
                heading = values['base_theta']
                dx, dy = pose.pose.position.x - values['base_x'], pose.pose.position.y - values['base_y']
                forward = math.cos(heading)*dx + math.sin(heading)*dy
                lateral = -math.sin(heading)*dx + math.cos(heading)*dy
                if not float(cfg.get('base_goal_tcp_x_min_m', 0.1)) <= forward <= float(cfg.get('base_goal_tcp_x_max_m', 1.0)) or abs(lateral) > float(cfg.get('base_goal_tcp_y_max_m', 0.2)):
                    rejected['local_reach'] = rejected.get('local_reach', 0) + 1
                    base_bounds = {c.joint_name: (round(c.position - c.tolerance_below, 3),
                                                  round(c.position + c.tolerance_above, 3))
                                   for c in constraints.joint_constraints
                                   if c.joint_name in BASE_JOINT_NAMES}
                    self.get_logger().info(
                        f'  local_reach reject #{index}: base=({values["base_x"]:.3f}, '
                        f'{values["base_y"]:.3f}, {heading:.3f}) forward={forward:.3f} '
                        f'lateral={lateral:.3f} target=({pose.pose.position.x:.3f}, '
                        f'{pose.pose.position.y:.3f}) base_bounds={base_bounds}')
                    continue
            # Explicitly rank base travel and avoid needless branch changes.
            score = sum((values[n] - current[n]) ** 2 * (20 if n in BASE_JOINT_NAMES else 1)
                        for n in names)
            accepted.append((score, candidate))
        self.get_logger().info(f'IK preflight {group}: {len(accepted)} valid, rejected={rejected}')
        if not accepted and last_request is not None:
            try:
                self._diagnose_continuation_ik(last_request, start, f'IK preflight {group}', 'goal')
            except Exception as exc:
                self.get_logger().warning(f'IK failure diagnostic unavailable: {exc}')
        accepted.sort(key=lambda item: item[0])
        return [q for _, q in accepted]

    def _validate_executed_path(self, raw_waypoints, cfg):
        # Validate the serialized positions, including adapter/resampling output
        # and interpolated intervals, rather than only the original OMPL path.
        group = cfg.get('group_name', self.default_group_name)
        base = deepcopy(self.latest_joint_state)
        path_constraints = cfg.get('_path_constraints', Constraints())
        checks = 0
        previous = raw_waypoints[0]
        for values in raw_waypoints:
            divisions = max(1, math.ceil(max(abs(a-b) / (0.005 if i < 2 else 0.02)
                                            for i, (a,b) in enumerate(zip(previous,values)))))
            checks += divisions
            if checks > 3000:
                raise RuntimeError('Trajectory validation budget exceeded; split the planning task')
            for k in range(1, divisions+1):
                q = {name: a+(b-a)*k/divisions for name,a,b in zip(TRAJECTORY_JOINT_ORDER,previous,values)}
                js = deepcopy(base)
                js.position = [q.get(n,p) for n,p in zip(js.name,js.position)]
                if not self._state_valid(js, group, path_constraints, log=True):
                    return False
            previous = values
        if '_goal_constraints' in cfg and not self._state_valid(js, group, cfg['_goal_constraints'], log=True):
            return False
        return True

    def _execute_preplanned(self, name, trajectory, cfg):
        # Share approval, validation and transport with MoveGroup results.
        with self._plan_lock:
            if self._active_plan is not None:
                raise RuntimeError('A planning request is already active')
            context = PlanContext(self._next_plan_request_id, name)
            self._next_plan_request_id += 1
            self._active_plan = context
        result = MoveGroup.Result()
        result.error_code.val = 1
        result.planned_trajectory = trajectory
        result.trajectory_start.joint_state = deepcopy(self.latest_joint_state)
        result.trajectory_start.is_diff = True
        future = Future()
        future.set_result(SimpleNamespace(result=result))
        self._goal_result_callback(context, future, cfg)
        return context

    def _plan_cartesian(self, name, pose, cfg, motion_request):
        req = GetCartesianPath.Request()
        req.header = pose.header
        req.start_state = motion_request.start_state
        req.group_name = motion_request.group_name
        req.link_name = cfg.get('link_name', self.end_effector_link)
        req.waypoints = [pose.pose]
        req.max_step = 0.005
        req.jump_threshold = 2.0
        req.revolute_jump_threshold = 0.15
        req.prismatic_jump_threshold = 0.01
        req.avoid_collisions = True
        req.path_constraints = motion_request.path_constraints
        result = self._call_service(self.cartesian_client, req, timeout=10.0)
        if result.error_code.val != 1 or result.fraction < 0.999999:
            self.get_logger().error(f'{name}: Cartesian approach incomplete ({result.fraction:.1%}); no motion sent')
            return None
        jt = result.solution.joint_trajectory
        if not jt.points:
            return None
        # Cartesian service does not guarantee timing. Use conservative segment
        # timing with zero-velocity endpoints; hardware interpolates positions.
        elapsed = 0.0
        previous = list(jt.points[0].positions)
        for i, point in enumerate(jt.points):
            delta = max(abs(a - b) for a, b in zip(previous, point.positions))
            if i:
                elapsed += max(0.1, 1.875 * delta / 0.35, math.sqrt(5.8 * delta / 0.25))
            point.time_from_start = Duration(seconds=elapsed).to_msg()
            point.velocities = [0.0] * len(point.positions)
            point.accelerations = [0.0] * len(point.positions)
            previous = list(point.positions)
        return self._execute_preplanned(name, result.solution, cfg)

    def _prepare_mobile_standoff(self, name, pose, cfg, request, candidates):
        """Choose a base pose with a feasible fixed-base continuation, then move it."""
        arm_group = 'press_arm' if cfg.get('link_name') == 'press_tcp' else 'arm'
        selected = None
        rejections = {}
        diagnosed = False
        def rejected(reason):
            rejections[reason] = rejections.get(reason, 0) + 1
        for candidate in candidates:
            lookahead_poses = []
            for waypoint_index, offset in enumerate(cfg.get('lookahead_offsets', [])):
                target = deepcopy(pose)
                resolved = self._resolve_detected_pose({'offset_x': offset[0], 'offset_y': offset[1],
                    'offset_z': offset[2], 'offset_frame': 'target', 'group_name': arm_group,
                    'frame_id': pose.header.frame_id}, pose)
                target.pose.position.x, target.pose.position.y, target.pose.position.z = resolved['position']
                lookahead_poses.append(deepcopy(target.pose))
            if lookahead_poses:
                path = GetCartesianPath.Request()
                path.header = pose.header
                path.start_state.joint_state = candidate
                path.start_state.is_diff = True
                path.group_name = arm_group
                path.link_name = cfg.get('link_name', self.end_effector_link)
                path.waypoints = lookahead_poses
                path.max_step = 0.005
                path.jump_threshold = 2.0
                path.revolute_jump_threshold = 0.15
                path.prismatic_jump_threshold = 0.01
                path.avoid_collisions = True
                path.path_constraints.orientation_constraints = deepcopy(request.goal_constraints[0].orientation_constraints)
                continuation = self._call_service(self.cartesian_client, path, timeout=10.0)
                if continuation.error_code.val != 1 or continuation.fraction < 0.999999:
                    rejected(f'Cartesian continuation: code={continuation.error_code.val}, fraction={continuation.fraction:.3f}')
                    if not diagnosed:
                        diagnosed = True
                        self._diagnose_cartesian_failure(name, path, continuation)
                    continue
            q = dict(zip(candidate.name, candidate.position))
            goal = {n: q[n] for n in BASE_JOINT_NAMES}
            phases = [(f'{name}/base', goal)]
            if (cfg.get('base_placement') or {}).get('base_motion') == 'turn_then_drive':
                # Turn in place, then drive: with the placement heading the
                # drive is straight ahead, so the wheels never turn and
                # translate at once. Both phases are checked before moving.
                here = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
                turn = {'base_x': here['base_x'], 'base_y': here['base_y'], 'base_theta': goal['base_theta']}
                phases = [(f'{name}/base_turn', turn), (f'{name}/base_drive', goal)]
            trajectory = self._plan_joint_phase(phases[0][0], 'base', self.latest_joint_state, phases[0][1], {})
            if trajectory is not None and len(phases) > 1:
                after_turn = self._joint_state_with_updates(phases[0][1])
                if self._plan_joint_phase(phases[1][0], 'base', after_turn, phases[1][1], {}) is None:
                    trajectory = None
            if trajectory is None:
                rejected('base path planning failed')
                continue
            selected = trajectory
            break
        if selected is None:
            self.get_logger().error(f'{name}: no base candidate supports the collision-checked fixed-base approach; rejections={rejections}; no motion sent')
            return None
        base_cfg = {k: v for k, v in cfg.items() if k not in ('_goal_constraints', '_path_constraints')}
        for index, (label, target) in enumerate(phases):
            if index:
                # Later phases start from the measured state the previous one reached.
                selected = self._plan_joint_phase(label, 'base', self.latest_joint_state, target, {})
                if selected is None:
                    self.stop_pub.publish(Bool(data=True))
                    raise RuntimeError(f'{label}: no base path from the measured state')
            if '_placement_state' in cfg:
                cfg['_placement_state']['base_moved'] = True
            context = self._execute_preplanned(label, selected, {**base_cfg, 'group_name': 'base'})
            if not context.success or not self._wait_for_arduino_done(90.0, label):
                self.stop_pub.publish(Bool(data=True))
                raise RuntimeError(f'{name}: base positioning failed')
        measured = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        dx, dy = measured['base_x'] - goal['base_x'], measured['base_y'] - goal['base_y']
        c, sn = math.cos(goal['base_theta']), math.sin(goal['base_theta'])
        self.get_logger().info(
            f'{name}: base planned ({goal["base_x"]:.4f}, {goal["base_y"]:.4f}, {goal["base_theta"]:.4f}) '
            f'measured ({measured["base_x"]:.4f}, {measured["base_y"]:.4f}, {measured["base_theta"]:.4f}): '
            f'error forward {(c * dx + sn * dy) * 1000:+.1f} mm, sideways {(-sn * dx + c * dy) * 1000:+.1f} mm, '
            f'heading {math.degrees(wrap(measured["base_theta"] - goal["base_theta"])):+.2f} deg (odometry)')
        # Replan the arm from actual reached base state, not the candidate state.
        arm_cfg = {**cfg, 'group_name': arm_group, 'mobile_standoff': False,
                   '_preferred_arm_state': deepcopy(candidate)}
        for key in ('base_goal_tcp_x_min_m', 'base_goal_tcp_x_max_m', 'base_goal_tcp_y_max_m',
                    'max_base_goal_rotation_rad', 'max_base_goal_translation_m',
                    '_placement_candidates', '_placement_state', 'base_placement'):
            arm_cfg.pop(key, None)
        if '_placement_target' in cfg:
            # The arm cannot yaw or strafe: aim it along the reached base
            # heading, inside the reached arm plane.
            resolved = self._resolve_in_arm_plane(arm_cfg, cfg['_placement_target'])
            if resolved is None:
                raise RuntimeError(f'{name}: pre-grasp unreachable from the reached base pose')
            arm_cfg = resolved
            pose = self._build_pose_message(resolved)
        return self._plan_pose(name + '/arm', pose, arm_cfg)

    def _diagnose_cartesian_failure(self, name, path, failed):
        """Read-only probes and scene capture. Never return a trajectory to execute."""
        try:
            get = GetPlanningScene.Request()
            get.components.components = 1023  # All PlanningSceneComponents.
            scene = self._call_service(self.scene_client, get, timeout=10.0).scene
            directory = Path(tempfile.mkdtemp(prefix='homeplus_planning_failure_'))
            (directory / 'scene.cdr').write_bytes(serialize_message(scene))
            (directory / 'request.cdr').write_bytes(serialize_message(path))
            (directory / 'response.cdr').write_bytes(serialize_message(failed))
            self.get_logger().warning(f'{name}: saved read-only planning replay to {directory}')
            probe = deepcopy(path)
            probe.avoid_collisions = False
            result = self._call_service(self.cartesian_client, probe, timeout=10.0)
            self.get_logger().warning(f'{name}: diagnostic collision-disabled fraction={result.fraction:.3f} '
                                      f'(original={failed.fraction:.3f}); diagnostic path will not execute')
            (directory / 'diagnostic_response.cdr').write_bytes(serialize_message(result))
            joints = deepcopy(path.start_state.joint_state)
            for index, point in enumerate(result.solution.joint_trajectory.points):
                updates = dict(zip(result.solution.joint_trajectory.joint_names, point.positions))
                joints.position = [updates.get(n, q) for n, q in zip(joints.name, joints.position)]
                if not self._state_valid(joints, path.group_name, path.path_constraints, log=True):
                    self.get_logger().error(f'{name}: first invalid diagnostic waypoint={index}; '
                                           'see collision pairs/constraint results above')
                    return
            self.get_logger().warning(f'{name}: no invalid sampled state found in diagnostic path; '
                                      'inspect replay for IK, jump checks, or changing map')
        except Exception as exc:
            self.get_logger().warning(f'{name}: planning diagnostic unavailable: {exc}')

    def _diagnose_continuation_ik(self, request, candidate, name, waypoint):
        """Explain a failed IK validity filter; never return a motion candidate."""
        probe = deepcopy(request)
        probe.ik_request.avoid_collisions = False
        probe.ik_request.constraints = Constraints()
        probe.ik_request.timeout = Duration(seconds=1.0).to_msg()
        result = self._call_service(self.ik_client, probe)
        q = dict(zip(candidate.name, candidate.position))
        prefix = (f'{name} waypoint {waypoint}: diagnostic at base '
                  f'x={q.get("base_x", 0):.3f}, y={q.get("base_y", 0):.3f}, '
                  f'yaw={q.get("base_theta", 0):.3f}')
        if result.error_code.val != 1:
            self.get_logger().warning(prefix + ': IK also failed without collision/constraint filters; '
                                      'candidate geometry or solver convergence is the blocker')
            return
        valid = self._state_valid(result.solution.joint_state,
            request.ik_request.group_name, request.ik_request.constraints, log=True)
        self.get_logger().warning(prefix + (': unconstrained solution fails scene/constraints; see contacts above'
            if not valid else ': solution passes scene/constraints; original IK search failed to find it'))

    def _plan_pose(
        self, name: str, pose_msg: PoseStamped, command_cfg: Dict
    ) -> Optional[PlanContext]:
        if self.move_group_client is None:
            return None
        if not self.move_group_client.server_is_ready():
            self.get_logger().warn("MoveIt action server is not ready.")
            return None
        if not self._clear_and_rebuild_octomap(name, command_cfg):
            return None

        self._require_fresh_state()
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
        goal.request.start_state.is_diff = True
        if self.latest_joint_state is not None:
            goal.request.start_state.joint_state = deepcopy(self.latest_joint_state)

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = pose_msg.header.frame_id
        position_constraint.link_name = str(command_cfg.get("link_name", self.end_effector_link))
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        tol = float(command_cfg.get("position_tolerance_m", self.position_tolerance_m))
        tol_xy = float(command_cfg.get("position_tolerance_xy_m", tol))
        tol_z = float(command_cfg.get("position_tolerance_z_m", tol))
        # SolidPrimitive dimensions are full side lengths, while the public
        # parameters are +/- tolerances around the requested point. Separate Z
        # tolerance prevents the planner from choosing an easier endpoint
        # several centimeters above a detected object.
        box.dimensions = [2.0 * tol_xy, 2.0 * tol_xy, 2.0 * tol_z]
        region_pose = Pose()
        region_pose.position = pose_msg.pose.position
        region_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitives = [box]
        position_constraint.constraint_region.primitive_poses = [region_pose]
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        use_orientation_constraint = bool(
            command_cfg.get("use_orientation_constraint", self.use_orientation_constraint)
        )
        if use_orientation_constraint:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = pose_msg.header.frame_id
            orientation_constraint.link_name = str(command_cfg.get("link_name", self.end_effector_link))
            orientation_constraint.orientation = pose_msg.pose.orientation
            if hasattr(orientation_constraint, "ROTATION_VECTOR"):
                orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
            orient_tol = float(
                command_cfg.get(
                    "orientation_tolerance_rad", self.orientation_tolerance_rad
                )
            )
            orient_tol_x = float(
                command_cfg.get("orientation_tolerance_x_rad", orient_tol)
            )
            orient_tol_y = float(
                command_cfg.get("orientation_tolerance_y_rad", orient_tol)
            )
            orient_tol_z = float(
                command_cfg.get("orientation_tolerance_z_rad", orient_tol)
            )
            orientation_constraint.absolute_x_axis_tolerance = orient_tol_x
            orientation_constraint.absolute_y_axis_tolerance = orient_tol_y
            orientation_constraint.absolute_z_axis_tolerance = orient_tol_z
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints.append(orientation_constraint)

            if bool(command_cfg.get("maintain_orientation_during_path", False)):
                path_constraints = Constraints()
                path_constraints.name = f"{name}_maintain_orientation"
                path_constraints.orientation_constraints.append(
                    deepcopy(orientation_constraint)
                )
                goal.request.path_constraints = path_constraints
                self.get_logger().info(
                    f"  Maintaining orientation throughout path for {name} "
                    f"(tolerances: roll={orient_tol_x:.3f}, "
                    f"pitch={orient_tol_y:.3f}, yaw={orient_tol_z:.3f} rad)"
                )

        self._add_local_base_goal_constraints(
            constraints, command_cfg, name, pose_msg
        )

        command_cfg = dict(command_cfg)
        command_cfg['_path_constraints'] = deepcopy(goal.request.path_constraints)
        command_cfg['_goal_constraints'] = deepcopy(constraints)
        self._require_fresh_state()
        if not self._state_valid(goal.request.start_state.joint_state, goal.request.group_name,
                                 goal.request.path_constraints, log=True):
            return None
        if '_placement_candidates' in command_cfg:
            candidates = self._validated_placements(pose_msg, command_cfg, constraints)
        else:
            candidates = self._ik_candidates(pose_msg, command_cfg, constraints)
        if not candidates:
            self.get_logger().error(f'{name}: no valid IK goal (reach, orientation, base bounds or collision); RRT not called')
            return None
        if command_cfg.get('mobile_standoff', False):
            goal.request.goal_constraints = [constraints]
            return self._prepare_mobile_standoff(name, pose_msg, command_cfg, goal.request, candidates)
        goal.request.goal_constraints = [self._joint_goal(q, goal.request.group_name) for q in candidates[:3]]
        goal.request.max_velocity_scaling_factor = float(command_cfg.get('velocity_scaling', 1.0))
        goal.request.max_acceleration_scaling_factor = float(command_cfg.get('acceleration_scaling', 1.0))
        if command_cfg.get('cartesian', False):
            return self._plan_cartesian(name, pose_msg, command_cfg, goal.request)
        self._fill_workspace(goal.request, pose_msg.header.frame_id)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        return self._send_move_group_goal(name, goal, command_cfg)

    def _joint_goal_already_reached(self, goal):
        self._require_fresh_state()
        if not self._hardware_ready or not goal.name or len(goal.name) != len(goal.position):
            return False
        measured = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        for name, value in zip(goal.name, goal.position):
            tolerance = 0.001 if name in ('base_x', 'base_y') else math.radians(0.5)
            if name not in measured or not math.isfinite(value) or not math.isfinite(measured[name]) or abs(measured[name] - value) > tolerance:
                return False
        return True

    def _plan_joint_state(
        self, name: str, joint_msg: JointState, command_cfg: Dict
    ) -> Optional[PlanContext]:
        if self.move_group_client is None:
            return None
        if not self.move_group_client.server_is_ready():
            self.get_logger().warn("MoveIt action server is not ready.")
            return None

        self._require_fresh_state()
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
        goal.request.start_state.is_diff = True
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
        if not self._state_valid(self.latest_joint_state, goal.request.group_name, log=True):
            return None
        target = deepcopy(self.latest_joint_state)
        update = dict(zip(joint_msg.name, joint_msg.position))
        target.position = [update.get(n, p) for n, p in zip(target.name, target.position)]
        if not self._state_valid(target, goal.request.group_name, log=True):
            return None
        goal.request.max_velocity_scaling_factor = float(command_cfg.get('velocity_scaling', 1.0))
        goal.request.max_acceleration_scaling_factor = float(command_cfg.get('acceleration_scaling', 1.0))
        self._fill_workspace(goal.request, str(command_cfg.get("frame_id", self.base_frame)))
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        return self._send_move_group_goal(name, goal, command_cfg)

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
    def _send_move_group_goal(
        self, name: str, goal: MoveGroup.Goal, command_cfg: Dict
    ) -> PlanContext:
        with self._plan_lock:
            if self._active_plan is not None:
                raise RuntimeError(
                    f"Cannot plan {name}: {self._active_plan.name} is still active"
                )
            request_id = self._next_plan_request_id
            self._next_plan_request_id += 1
            context = PlanContext(request_id=request_id, name=name)
            self._active_plan = context

        planner_id = goal.request.planner_id or "group default"
        self._publish_status(f"Planning {name} with {planner_id} [request {request_id}]")
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._goal_response_callback(context, done, command_cfg)
        )
        return context

    def _goal_response_callback(self, context: PlanContext, future, command_cfg: Dict) -> None:
        name = context.name
        try:
            goal_handle = future.result()
        except Exception as exc:
            if context.cancelled:
                return
            self.get_logger().error(f"MoveIt request for {name} failed before acceptance: {exc}")
            self._publish_status(f"MoveIt request for {name} failed: {exc}")
            self._finish_plan(context, False)
            return

        if not self._is_plan_context_active(context):
            self.get_logger().warn(f"Cancelling stale MoveIt response for {name}")
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"MoveIt request for {name} was rejected")
            self._publish_status(f"MoveIt request for {name} was rejected")
            self._finish_plan(context, False)
            return

        with self._plan_lock:
            if self._active_plan is context and not context.cancelled:
                context.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._goal_result_callback(context, done, command_cfg)
        )

    def _goal_result_callback(self, context: PlanContext, future, command_cfg: Dict) -> None:
        try:
            self._handle_goal_result(context, future, command_cfg)
        except Exception as exc:
            self.stop_pub.publish(Bool(data=True))
            self.get_logger().error(f'{context.name}: execution/validation failed: {exc}')
            self._finish_plan(context, False)

    def _handle_goal_result(self, context: PlanContext, future, command_cfg: Dict) -> None:
        name = context.name
        if not self._is_plan_context_active(context):
            self.get_logger().warn(f"Ignoring stale MoveIt result for {name}")
            return
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"MoveIt result retrieval failed for {name}: {exc}")
            self._publish_status(f"MoveIt result retrieval failed for {name}: {exc}")
            self._finish_plan(context, False)
            return

        error_code = int(result.error_code.val)
        if error_code == 1:
            self.get_logger().info(f"MoveIt planning succeeded for {name}")
            self._publish_status(f"MoveIt planning succeeded for {name}")

            trajectory = result.planned_trajectory
            if trajectory and trajectory.joint_trajectory.points:
                self._reset_trajectory_approval()
                require_approval = bool(command_cfg.get(
                    "require_trajectory_approval", self.require_trajectory_approval
                ))
                if require_approval:
                    with self._plan_lock:
                        if self._active_plan is context and not context.cancelled:
                            context.awaiting_approval = True
                self._publish_display_trajectory(name, result)
                extracted = self._extract_trajectory_waypoints(name, trajectory, command_cfg)
                if extracted is None:
                    self._finish_plan(context, False)
                    return
                jt, raw_waypoints, arduino_waypoints = extracted
                self._record_planned_trajectory(name, jt, raw_waypoints, arduino_waypoints)

                if require_approval:
                    if not self._wait_for_trajectory_approval(name, context):
                        self._publish_status(f"Plan for {name} was not approved")
                        self._finish_plan(context, False)
                        return
                elif self.require_trajectory_approval:
                    self.get_logger().info(
                        f"Plan for {name} is covered by the sequence approval; "
                        "sending to Arduino automatically"
                    )
                    self._publish_status(
                        f"Plan for {name} auto-approved by sequence configuration"
                    )

                if not self._is_plan_context_active(context):
                    self.get_logger().warn(f"Not executing stale plan for {name}")
                    return

                if not self._validate_executed_path(raw_waypoints, command_cfg):
                    self._finish_plan(context, False)
                    return
                if not self._is_plan_context_active(context):
                    return
                self._send_trajectory_to_arduino(name, raw_waypoints, arduino_waypoints,
                    (command_cfg or {}).get("endpoint_profile", "precise"))
            else:
                self.get_logger().warn(f"No trajectory in result for {name}")
                self._finish_plan(context, False)
                return

            self._finish_plan(context, True)
        else:
            error_name = MOVEIT_ERROR_NAMES.get(error_code, "UNKNOWN")
            failure = f"{error_name} ({error_code})"
            self.get_logger().warn(f"MoveIt failed for {name}: {failure}")
            self._publish_status(f"MoveIt failed for {name}: {failure}")
            context.retryable_failure = error_code in (-1, -6)
            self._finish_plan(context, False)

    def _is_plan_context_active(self, context: PlanContext) -> bool:
        with self._plan_lock:
            return self._active_plan is context and not context.cancelled

    def _finish_plan(self, context: PlanContext, success: bool) -> None:
        with self._plan_lock:
            if self._active_plan is not context or context.cancelled:
                return
            context.success = success
            context.awaiting_approval = False
            self._active_plan = None
        context.event.set()

    def _cancel_plan(self, context: PlanContext, reason: str) -> None:
        self.stop_pub.publish(Bool(data=True))
        goal_handle = None
        with self._plan_lock:
            if context.cancelled:
                return
            context.cancelled = True
            context.awaiting_approval = False
            goal_handle = context.goal_handle
            if self._active_plan is context:
                self._active_plan = None
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel MoveIt goal for {context.name}: {exc}")
        context.event.set()
        self.get_logger().warn(f"Cancelled plan for {context.name}: {reason}")
        self._publish_status(f"Cancelled plan for {context.name}: {reason}")

    def _wait_for_plan(self, context: PlanContext, command_cfg: Dict) -> bool:
        timeout = self._plan_wait_timeout(command_cfg)
        completed = context.event.wait(timeout=timeout)
        if not completed:
            timeout_text = "unbounded" if timeout is None else f"{timeout:.1f}s"
            self._cancel_plan(context, f"request exceeded {timeout_text}")
            return False
        return context.success and not context.cancelled

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
