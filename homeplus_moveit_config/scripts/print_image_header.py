#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class HeaderPrinter(Node):
    def __init__(self, topic):
        super().__init__('print_image_header')
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10)

    def listener_callback(self, msg: Image):
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        self.get_logger().info(f"Received header: frame_id='{msg.header.frame_id}' stamp={sec}.{nsec:09d}")
        rclpy.shutdown()


def main():
    import sys
    topic = '/camera/camera/depth/image_rect_raw' if len(sys.argv) < 2 else sys.argv[1]
    rclpy.init()
    node = HeaderPrinter(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
