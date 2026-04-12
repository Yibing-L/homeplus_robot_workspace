#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class CloudBuilder(Node):
    def __init__(self):
        super().__init__('cloud_builder')

        # Topics
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('mask_topic', '/object_mask')

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        self.bridge = CvBridge()

        self.mask = None

        self.camera_info_received = False

        # Publishers (IMPORTANT FOR MOVEIT)
        self.full_pub = self.create_publisher(PointCloud2, '/full_cloud', 10)
        self.object_pub = self.create_publisher(PointCloud2, '/object_cloud', 10)
        

        # Subscribers
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.mask_topic, self.mask_callback, 10)

        self.get_logger().info("CloudBuilder node ready")

    # ---------------------------
    # Camera intrinsics
    # ---------------------------
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)

            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]

            self.camera_info_received = True

            self.get_logger().info("Camera intrinsics received (locked)")

    # ---------------------------
    # SAM2 mask
    # ---------------------------
    def mask_callback(self, msg):
        self.mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')

    # ---------------------------
    # Depth → PointCloud
    # ---------------------------
    def depth_callback(self, msg):
        if not self.camera_info_received:
            return

        depth = self.bridge.imgmsg_to_cv2(msg)

        h, w = depth.shape

        full_points = []
        object_points = []

        step = 4

        for v in range(0, h, step):
            for u in range(0, w, step):

                if depth.dtype == np.uint16:
                    z = depth[v, u] / 1000.0
                else:
                    z = float(depth[v, u])
                if z == 0 or z > 5.0:
                    continue

                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                full_points.append([x, y, z])

                # SAM2 mask (optional)
                if self.mask is not None and self.mask[v, u] > 0:
                    object_points.append([x, y, z])

        header = msg.header

        # publish clouds
        full_cloud = pc2.create_cloud_xyz32(header, full_points)
        self.full_pub.publish(full_cloud)

        if len(object_points) > 0:
            object_cloud = pc2.create_cloud_xyz32(header, object_points)
            self.object_pub.publish(object_cloud)

def main():
    rclpy.init()
    node = CloudBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()