"""Fake robot speaking the *current* (unmodified) robot interfaces: STATE-rate joint states in SI,
latched ready/status, executor status strings + busy Bool, Bool approval, /display_planned_path,
detector diagnostics/poses, a camera image. Records what the console sends back."""
import json, math, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from homeplus_interfaces.msg import GestureCommand

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
NAMES = ["base_x", "base_y", "base_theta", "joint_grip_L", "joint_hand", "joint_wrist", "joint_elbow", "joint_shoulder"]
STEPS = ["detection", "open_gripper", "retract_arm_for_pickup", "move_to_pregrasp", "refine_target",
         "align_refined_grasp", "move_to_grasp", "close_gripper_at_target", "lift"]

class Bridge(Node):
    def __init__(self):
        super().__init__("arduino_bridge")
        self.js = self.create_publisher(JointState, "/hardware/joint_states", 10)
        self.ready = self.create_publisher(Bool, "/arduino/ready", LATCHED)
        self.status = self.create_publisher(Bool, "/arduino/status", LATCHED)
        self.stops = 0
        self.create_subscription(Bool, "/arduino/stop", self.on_stop, 10)
        self.ready.publish(Bool(data=True)); self.status.publish(Bool(data=True))
        self.get_logger().info("Connected /home/x/ttyArduino at 115200; waiting for firmware state feedback")
        self.create_timer(0.04, self.tick)
        self.create_timer(5.0, lambda: self.ready.publish(Bool(data=True)))
        self.create_timer(2.0, lambda: self.get_logger().info("STATE4 detected; waiting for measured stillness"))
    def on_stop(self, m):
        self.stops += 1
    def tick(self):
        js = JointState(); js.header.stamp = self.get_clock().now().to_msg()
        js.name = NAMES; js.position = [0.42, 0.0, 0.0, math.radians(40), 0.0, 0.0, math.radians(60), math.radians(30)]
        self.js.publish(js)

class Executor(Node):
    def __init__(self):
        super().__init__("gesture_motion_executor")
        self.status = self.create_publisher(String, "/gesture_control/executor_status", 10)
        self.busy = self.create_publisher(Bool, "/gesture_control/executor_busy", 10)
        self.disp = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.commands, self.approvals = [], []
        self.create_subscription(GestureCommand, "/gesture_control/command", self.on_cmd, 10)
        self.create_subscription(Bool, "/gesture_control/approve_planned_path", lambda m: self.approvals.append(m.data), 10)
    def say(self, t):
        self.status.publish(String(data=t)); self.get_logger().info(t)
    def on_cmd(self, m):
        self.commands.append((m.command_name, m.command_type, m.gesture_id, m.gesture_label))
        if m.command_name in ("pick_up_coffee", "gripper_open"):  # any command exercises the sequence path
            threading.Thread(target=self.run_seq, daemon=True).start()
    def run_seq(self):
        self.busy.publish(Bool(data=True)); self.say("Starting sequence: pick_up_coffee")
        for i, s in enumerate(STEPS[:6], 1):
            self.say(f"[pick_up_coffee] step {i}/9: {s}"); time.sleep(0.3)
        dt = DisplayTrajectory(); rt = RobotTrajectory(); jt = JointTrajectory(); jt.joint_names = NAMES
        for k in range(10):
            p = JointTrajectoryPoint(); u = k / 9
            p.positions = [0.42 + 0.03 * u, -0.01 * u, math.radians(1.2) * u, math.radians(40), 0.0, 0.0,
                           math.radians(60 + 11 * u), math.radians(30 + 8 * u)]
            t = 2.4 * u
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            jt.points.append(p)
        rt.joint_trajectory = jt; dt.trajectory.append(rt)
        self.disp.publish(dt)
        self.say("Plan for align_refined_grasp is waiting for approval")
        t0 = time.time()
        while not self.approvals and time.time() - t0 < 60: time.sleep(0.1)
        if self.approvals and self.approvals[-1]:
            self.say("Plan for align_refined_grasp approved; sending to Arduino")
            self.say("Sequence pick_up_coffee ABORTED at align_refined_grasp: Arduino execution error: ERROR 12 TRACKING_ERROR dx_cm=4.1")
        self.busy.publish(Bool(data=False))

class Vision(Node):
    def __init__(self):
        super().__init__("molmopoint")
        self.diag = self.create_publisher(String, "/molmopoint/diagnostics", 10)
        self.pose = self.create_publisher(PoseStamped, "/molmopoint/task_1/pose", 10)
        self.img = self.create_publisher(Image, "/cameras/d405/color/image_raw", 10)
        self.depth = self.create_publisher(Image, "/cameras/d405/aligned_depth_to_color/image_raw", 10)
        self.declare_parameter("task_id", 1)
        self.n = 0
        self.create_timer(1.0, self.cycle)
        self.create_timer(0.2, self.frame)
    def cycle(self):
        self.diag.publish(String(data=json.dumps({"stage": "done", "reason": "pose_published", "frame_age_sec": 0.2, "task_id": 1})))
        p = PoseStamped(); p.header.frame_id = "world"; p.pose.position.x, p.pose.position.y, p.pose.position.z = 0.95, 0.02, 0.76
        self.pose.publish(p)
    def frame(self):
        self.n += 1
        a = np.full((60, 80, 3), (self.n * 7) % 255, dtype=np.uint8)
        m = Image(); m.height, m.width, m.encoding, m.step = 60, 80, "rgb8", 240; m.data = a.tobytes()
        self.img.publish(m)
        d = Image(); d.height, d.width, d.encoding, d.step = 60, 80, "16UC1", 160
        d.data = (np.full((60, 80), 500 + self.n % 50, dtype=np.uint16)).tobytes()
        self.depth.publish(d)

class ObstacleCloud(Node):
    """Logs the real d405_obstacle_cloud warning while /tmp/tf_drop exists."""
    def __init__(self):
        super().__init__("d405_obstacle_cloud")
        self.create_timer(0.5, self.tick)
    def tick(self):
        import os
        if os.path.exists("/tmp/tf_drop"):
            self.get_logger().warning("Dropping depth frame: acquisition-time TF still missing after 0.5 s")

def main():
    rclpy.init()
    nodes = [Executor(), Vision(), ObstacleCloud()]
    ex = rclpy.executors.MultiThreadedExecutor()
    for n in nodes: ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    while True:
        time.sleep(1)
        with open("/tmp/fake_robot.json.tmp", "w") as f:
            json.dump({"commands": nodes[0].commands, "approvals": nodes[0].approvals}, f)
        __import__("os").replace("/tmp/fake_robot.json.tmp", "/tmp/fake_robot.json")

if __name__ == "__main__":
    main()
