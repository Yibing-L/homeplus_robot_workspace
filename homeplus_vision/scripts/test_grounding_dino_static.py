#!/usr/bin/env python3
"""
Static test publisher for the Grounding DINO node.

This lightweight script repeatedly publishes a color image and a matching CameraInfo
so you can test `grounding_dino_node.py` without running the RealSense driver.

Usage:
    python3 test_grounding_dino_static.py --image /path/to/test.jpg --rate 1 --count 10

Parameters:
- --image: required path to an image file (jpg/png)
- --rate: publish rate in Hz (default 1)
- --count: number of frames to publish (default 10). Use 0 for infinite.
- --image-topic: topic to publish the color image (default: /camera/camera/color/image_raw)
- --camera-info-topic: topic to publish CameraInfo (default: /camera/camera/color/camera_info)
- --frame-id: camera frame id to set in headers (default: camera_color_optical_frame)

It publishes sensor_msgs/Image and sensor_msgs/CameraInfo messages using CvBridge.
"""

import argparse
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class StaticImagePublisher(Node):
    def __init__(self, image_path, rate_hz=1.0, count=10, image_topic='/camera/camera/color/image_raw', camera_info_topic='/camera/camera/color/camera_info', frame_id='camera_color_optical_frame'):
        super().__init__('gdino_test_publisher')
        self.bridge = CvBridge()
        self.image = cv2.imread(image_path)
        if self.image is None:
            raise RuntimeError(f'Failed to load image: {image_path}')
        h, w = self.image.shape[:2]
        # Minimal CameraInfo with focal lengths roughly fx=fy=0.5*width
        fx = fy = 0.5 * w
        cx = w / 2.0
        cy = h / 2.0

        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = frame_id
        self.camera_info.width = w
        self.camera_info.height = h
        # K matrix in row-major
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        # P matrix
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.cinfo_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self.rate_hz = float(rate_hz)
        self.count = int(count)

    def run(self):
        r = self.get_clock()
        sent = 0
        try:
            while True:
                now = r.now().to_msg()
                # Image
                img_msg = self.bridge.cv2_to_imgmsg(self.image, 'bgr8')
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.camera_info.header.frame_id
                # CameraInfo
                self.camera_info.header.stamp = now
                # publish
                self.image_pub.publish(img_msg)
                self.cinfo_pub.publish(self.camera_info)
                self.get_logger().info(f'Published test image/frame {sent+1}')
                sent += 1
                if self.count > 0 and sent >= self.count:
                    break
                time.sleep(1.0 / max(0.0001, self.rate_hz))
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True, help='Path to test image (jpg/png)')
    parser.add_argument('--rate', type=float, default=1.0, help='Publish rate Hz')
    parser.add_argument('--count', type=int, default=10, help='Number of frames to publish (0 = infinite)')
    parser.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--frame-id', default='camera_color_optical_frame')

    args = parser.parse_args()

    rclpy.init()
    node = StaticImagePublisher(args.image, rate_hz=args.rate, count=args.count, image_topic=args.image_topic, camera_info_topic=args.camera_info_topic, frame_id=args.frame_id)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
