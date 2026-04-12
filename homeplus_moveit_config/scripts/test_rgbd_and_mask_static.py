#!/usr/bin/env python3
"""
Publish static RGB + depth + CameraInfo and optional mask to test cloud_builder and MoveIt pipeline.

Usage examples:
  # publish color + depth + camera_info + mask forever (rate 1Hz)
  python3 test_rgbd_and_mask_static.py --color /path/to/color.jpg --depth /path/to/depth.png --mask /path/to/mask.png --rate 1

  # publish only depth+camera_info (no color)
  python3 test_rgbd_and_mask_static.py --depth /path/to/depth.png --rate 2 --count 10

Notes:
- Depth image may be a 16-bit PNG (values in millimeters) or an 8-bit image (will be scaled).
- The script publishes:
    color -> /camera/camera/color/image_raw
    color camera info -> /camera/camera/color/camera_info
    depth -> /camera/camera/depth/image_rect_raw
    depth camera info -> /camera/camera/depth/camera_info
    mask -> /object_mask

These topics match the defaults used by cloud_builder in this workspace.
"""
import argparse
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class StaticRGBDPublisher(Node):
    def __init__(self, color_path=None, depth_path=None, mask_path=None, rate=1.0, count=0,
                 color_topic='/camera/camera/color/image_raw', color_info_topic='/camera/camera/color/camera_info',
                 depth_topic='/camera/camera/depth/image_rect_raw', depth_info_topic='/camera/camera/depth/camera_info',
                 mask_topic='/object_mask', frame_id='camera_color_optical_frame'):
        super().__init__('rgbd_test_publisher')
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.mask = None
        self.frame_id = frame_id

        if color_path:
            img = cv2.imread(color_path, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f'Failed to load color image: {color_path}')
            self.color = img
        if depth_path:
            dep = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if dep is None:
                raise RuntimeError(f'Failed to load depth image: {depth_path}')
            # dep may be 16-bit or 8-bit; normalize/keep dtype
            self.depth = dep
        if mask_path:
            m = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            if m is None:
                raise RuntimeError(f'Failed to load mask image: {mask_path}')
            self.mask = m

        # Determine a reference size from whichever image exists
        ref = self.color if self.color is not None else self.depth
        if ref is None:
            raise RuntimeError('At least one of --color or --depth must be provided')
        h, w = ref.shape[:2]

        # Build a simple CameraInfo
        fx = fy = 0.5 * w
        cx = w / 2.0
        cy = h / 2.0
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = int(w)
        self.camera_info.height = int(h)
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        qos_depth = 10
        self.color_pub = self.create_publisher(Image, color_topic, 10)
        self.color_info_pub = self.create_publisher(CameraInfo, color_info_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_topic, qos_depth)
        self.depth_info_pub = self.create_publisher(CameraInfo, depth_info_topic, 10)
        self.mask_pub = self.create_publisher(Image, mask_topic, 10)

        self.rate = float(rate)
        self.count = int(count)

    def _make_depth_msg(self, arr):
        # arr is numpy array, possibly uint16 or uint8
        msg = Image()
        if arr.dtype == np.uint16:
            msg.encoding = '16UC1'
            # assume units are millimeters in file
            msg.data = arr.tobytes()
            msg.step = arr.shape[1] * 2
        else:
            # convert to uint16 by scaling 0-255 -> 0-5000 mm
            arr16 = (arr.astype(np.float32) / 255.0 * 5000.0).astype(np.uint16)
            msg.encoding = '16UC1'
            msg.data = arr16.tobytes()
            msg.step = arr16.shape[1] * 2
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.is_bigendian = 0
        return msg

    def _make_color_msg(self, arr):
        return self.bridge.cv2_to_imgmsg(arr, encoding='bgr8')

    def _make_mask_msg(self, arr):
        return self.bridge.cv2_to_imgmsg(arr, encoding='mono8')

    def run(self):
        clock = self.get_clock()
        sent = 0
        try:
            while True:
                now = clock.now().to_msg()
                # color
                if self.color is not None:
                    img_msg = self._make_color_msg(self.color)
                    img_msg.header.stamp = now
                    img_msg.header.frame_id = self.frame_id
                    self.color_pub.publish(img_msg)
                    self.camera_info.header.stamp = now
                    self.color_info_pub.publish(self.camera_info)
                # depth
                if self.depth is not None:
                    dep_msg = self._make_depth_msg(self.depth)
                    dep_msg.header.stamp = now
                    dep_msg.header.frame_id = self.frame_id
                    self.depth_pub.publish(dep_msg)
                    self.depth_info_pub.publish(self.camera_info)
                # mask
                if self.mask is not None:
                    mask_msg = self._make_mask_msg(self.mask)
                    mask_msg.header.stamp = now
                    mask_msg.header.frame_id = self.frame_id
                    self.mask_pub.publish(mask_msg)

                self.get_logger().info(f'Published RGBD frame {sent+1}')
                sent += 1
                if self.count > 0 and sent >= self.count:
                    break
                time.sleep(1.0 / max(0.0001, self.rate))
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default=None)
    parser.add_argument('--depth', default=None)
    parser.add_argument('--mask', default=None)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--count', type=int, default=0)
    parser.add_argument('--frame-id', default='camera_color_optical_frame')
    args = parser.parse_args()

    rclpy.init()
    node = StaticRGBDPublisher(color_path=args.color, depth_path=args.depth, mask_path=args.mask,
                               rate=args.rate, count=args.count, frame_id=args.frame_id)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
