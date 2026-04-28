#!/usr/bin/env python3
"""
Listens for Arduino status message (default topic `/arduino/status`). When a
"done" status is received the node calls the octomap reset service and
re-publishes the most recent depth + CameraInfo messages a few times to trigger
cloud_builder -> octomap insertion.

Run:
  ros2 run homeplus_moveit_config rebuild_map.py

Parameters:
    arduino_topic: topic to listen for status (std_msgs/Bool) (default: /arduino/status)
  depth_topic: topic where depth images arrive (default: /camera/camera/depth/image_rect_raw)
  camera_info_topic: CameraInfo topic (default: /camera/camera/depth/camera_info)
  republish_count: how many times to republish buffered frames after reset (default: 3)
  republish_delay: seconds between republished frames (default: 0.2)
"""
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty
from std_msgs.msg import Bool


class RebuildMap(Node):
    def __init__(self):
        super().__init__('rebuild_map')

        # Params
        self.declare_parameter('arduino_topic', '/arduino/status')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('republish_count', 3)
        self.declare_parameter('republish_delay', 0.2)

        self.arduino_topic = self.get_parameter('arduino_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.republish_count = int(self.get_parameter('republish_count').value)
        self.republish_delay = float(self.get_parameter('republish_delay').value)

        # Storage for latest frames
        self.latest_depth: Optional[Image] = None
        self.latest_cinfo: Optional[CameraInfo] = None

        # Subscribers
        self.create_subscription(Image, self.depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._cinfo_cb, qos_profile_sensor_data)

        # Subscribe to Arduino status
        self.create_subscription(Bool, self.arduino_topic, self._arduino_cb, 10)

        # Publishers
        self.depth_pub = self.create_publisher(Image, self.depth_topic, qos_profile_sensor_data)
        self.cinfo_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        # Service client to reset octomap
        self._reset_client = self.create_client(Empty, '/octomap_server/reset')

        self.get_logger().info(f'RebuildMap listening on {self.arduino_topic}; will republish {self.republish_count} frames')

    def _depth_cb(self, msg: Image):
        self.latest_depth = msg

    def _cinfo_cb(self, msg: CameraInfo):
        self.latest_cinfo = msg

    def _arduino_cb(self, msg: Bool):
        # Arduino publishes a Bool: True means 'done'
        try:
            val = bool(msg.data)
        except Exception:
            self.get_logger().warning('Received malformed arduino status message')
            return

        self.get_logger().info(f'Received arduino status: {val}')
        if val:
            self.get_logger().info('Arduino reported done — rebuilding octomap')
            self._trigger_rebuild()

    def _wait_for_reset_service(self, timeout_sec=5.0) -> bool:
        if self._reset_client.wait_for_service(timeout_sec=timeout_sec):
            return True
        else:
            self.get_logger().warning('octomap reset service not available')
            return False

    def _call_reset(self) -> bool:
        if not self._wait_for_reset_service(2.0):
            return False
        req = Empty.Request()
        fut = self._reset_client.call_async(req)
        # wait a short while
        timeout = 3.0
        t0 = self.get_clock().now().nanoseconds
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now().nanoseconds - t0) / 1e9 > timeout:
                self.get_logger().warning('Timed out waiting for octomap reset response')
                return False
        try:
            res = fut.result()
            self.get_logger().info('octomap reset service succeeded')
            return True
        except Exception as e:
            self.get_logger().error(f'octomap reset call failed: {e}')
            return False

    def _republish_buffered(self):
        if self.latest_depth is None or self.latest_cinfo is None:
            self.get_logger().warning('No buffered depth/CameraInfo available to republish')
            return

        for i in range(self.republish_count):
            now = self.get_clock().now().to_msg()
            # update stamps (preserve frame_id)
            d = self.latest_depth
            c = self.latest_cinfo
            d.header.stamp = now
            c.header.stamp = now
            try:
                self.depth_pub.publish(d)
                self.cinfo_pub.publish(c)
                self.get_logger().info(f'Republished buffered frame {i+1}/{self.republish_count}')
            except Exception as e:
                self.get_logger().error(f'Failed to republish buffered frames: {e}')
            time.sleep(self.republish_delay)

    def _trigger_rebuild(self):
        ok = self._call_reset()
        if not ok:
            self.get_logger().warning('Proceeding to republish frames even though reset failed/unavailable')

        self._republish_buffered()


def main():
    rclpy.init()
    node = RebuildMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
