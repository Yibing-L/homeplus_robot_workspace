#!/usr/bin/env python3

from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from homeplus_interfaces.msg import GestureCommand, GestureRecognition
from rclpy.node import Node
from std_msgs.msg import String


class GestureBehaviorNode(Node):
    def __init__(self):
        super().__init__("gesture_behavior")

        self.declare_parameter("behavior_map_path", "")
        self.declare_parameter("recognition_topic", "/gesture_recognition/state")
        self.declare_parameter("command_topic", "/gesture_control/command")
        self.declare_parameter("status_topic", "/gesture_control/behavior_status")
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("min_trigger_interval_sec", 1.5)
        self.declare_parameter("require_active", True)

        behavior_map_path = str(self.get_parameter("behavior_map_path").value)
        if not behavior_map_path:
            raise ValueError("Parameter 'behavior_map_path' is required.")
        if not Path(behavior_map_path).is_file():
            raise FileNotFoundError(f"Behavior map not found: {behavior_map_path}")

        self.behaviors = self._load_behavior_map(behavior_map_path)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.min_trigger_interval_sec = float(self.get_parameter("min_trigger_interval_sec").value)
        self.require_active = bool(self.get_parameter("require_active").value)

        self.command_pub = self.create_publisher(
            GestureCommand,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            GestureRecognition,
            str(self.get_parameter("recognition_topic").value),
            self.recognition_callback,
            10,
        )

        self.latched_gesture_id: Optional[int] = None
        self.last_trigger_time: Optional[rclpy.time.Time] = None

        self.get_logger().info(
            f"Gesture behavior ready with {len(self.behaviors)} mappings from {behavior_map_path}"
        )

    def _load_behavior_map(self, path: str) -> Dict[int, Dict]:
        with open(path, "r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        behaviors = {}
        for entry in payload.get("behaviors", []):
            gesture_id = int(entry["gesture_id"])
            behaviors[gesture_id] = entry
        if not behaviors:
            raise ValueError(f"No behaviors found in {path}")
        return behaviors

    def recognition_callback(self, msg: GestureRecognition) -> None:
        gesture_id = int(msg.class_id)
        if gesture_id < 0:
            self.latched_gesture_id = None
            return
        if self.require_active and not msg.active:
            self.latched_gesture_id = None
            return

        behavior = self.behaviors.get(gesture_id)
        threshold = self.min_confidence
        if behavior is not None and behavior.get("min_confidence") is not None:
            threshold = float(behavior["min_confidence"])
        if float(msg.confidence) < threshold:
            self.latched_gesture_id = None
            return
        if self.latched_gesture_id == gesture_id:
            return

        now = self.get_clock().now()
        if self.last_trigger_time is not None:
            elapsed_sec = (now - self.last_trigger_time).nanoseconds / 1e9
        else:
            elapsed_sec = self.min_trigger_interval_sec
        if elapsed_sec < self.min_trigger_interval_sec:
            return

        if behavior is None:
            self._publish_status(f"No behavior configured for gesture {gesture_id}")
            self.get_logger().warn(f"No behavior configured for gesture {gesture_id}")
            self.latched_gesture_id = gesture_id
            self.last_trigger_time = now
            return

        command = GestureCommand()
        command.header = msg.header
        if command.header.stamp.sec == 0 and command.header.stamp.nanosec == 0:
            command.header.stamp = now.to_msg()
        command.gesture_id = gesture_id
        command.gesture_label = msg.label
        command.confidence = float(msg.confidence)
        command.command_name = str(behavior["command_name"])
        command.command_type = str(behavior.get("command_type", ""))

        self.command_pub.publish(command)
        self._publish_status(
            f"Published command={command.command_name} from gesture={gesture_id} "
            f"label={msg.label} conf={msg.confidence:.2f}"
        )
        self.get_logger().info(
            f"gesture={gesture_id} label={msg.label} -> command={command.command_name}"
        )

        self.latched_gesture_id = gesture_id
        self.last_trigger_time = now

    def _publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureBehaviorNode()
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
