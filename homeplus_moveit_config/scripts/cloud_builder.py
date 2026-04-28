#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener, TransformException




class CloudBuilder(Node):
    def __init__(self):
        super().__init__('cloud_builder')

        # Topics
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('mask_topic', '/object_mask')
        self.declare_parameter('output_frame', '')

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mask = None
        self.last_mask_shape_warning = None
        self.last_transform_warning = None

        self.camera_info_received = False

        # Publishers (IMPORTANT FOR MOVEIT)
        self.full_pub = self.create_publisher(PointCloud2, '/full_cloud', 10)
        self.object_pub = self.create_publisher(PointCloud2, '/object_cloud', 10)

        # Subscribers
        # RealSense publishes sensor topics with sensor-data QoS; match that profile
        # so depth and camera-info subscriptions reliably connect.
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.mask_topic, self.mask_callback, qos_profile_sensor_data)

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
        source_frame = msg.header.frame_id or 'hand_camera_depth_optical_frame'
        transform = None
        if self.output_frame and self.output_frame != source_frame:
            transform = self._lookup_transform(self.output_frame, source_frame, msg.header.stamp)
            if transform is None:
                return

        h, w = depth.shape
        mask_for_depth = self.mask
        if mask_for_depth is not None and mask_for_depth.shape != (h, w):
            # Keep mask/depth aligned even if upstream image sizes differ.
            if self.last_mask_shape_warning != (mask_for_depth.shape, (h, w)):
                self.get_logger().warning(
                    f"Mask/depth shape mismatch (mask={mask_for_depth.shape}, depth={(h, w)}). "
                    "Resizing mask to depth size."
                )
                self.last_mask_shape_warning = (mask_for_depth.shape, (h, w))
            import cv2
            mask_for_depth = cv2.resize(self.mask, (w, h), interpolation=cv2.INTER_NEAREST)

        step = 4

        # Downsample depth (and mask) by step
        depth_ds = depth[::step, ::step]
        if depth_ds.dtype == np.uint16:
            z = depth_ds.astype(np.float32) / 1000.0
        else:
            z = depth_ds.astype(np.float32)

        # Build pixel coordinate grids
        v_indices, u_indices = np.mgrid[0:h:step, 0:w:step].astype(np.float32)

        # Validity mask: finite, positive, within range
        valid = np.isfinite(z) & (z > 0.0) & (z <= 5.0)

        z_valid = z[valid]
        u_valid = u_indices[valid]
        v_valid = v_indices[valid]

        x_valid = (u_valid - self.cx) * z_valid / self.fx
        y_valid = (v_valid - self.cy) * z_valid / self.fy

        full_points = np.column_stack([x_valid, y_valid, z_valid])

        # SAM2 mask (optional)
        object_points = np.empty((0, 3), dtype=np.float32)
        if mask_for_depth is not None:
            mask_ds = mask_for_depth[::step, ::step]
            obj_mask = valid & (mask_ds > 0)
            if np.any(obj_mask):
                z_obj = z[obj_mask]
                u_obj = u_indices[obj_mask]
                v_obj = v_indices[obj_mask]
                x_obj = (u_obj - self.cx) * z_obj / self.fx
                y_obj = (v_obj - self.cy) * z_obj / self.fy
                object_points = np.column_stack([x_obj, y_obj, z_obj])

        if transform is not None:
            full_points = self._transform_points(full_points, transform)
            if len(object_points) > 0:
                object_points = self._transform_points(object_points, transform)

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        if self.output_frame:
            header.frame_id = self.output_frame
        elif not header.frame_id:
            header.frame_id = 'hand_camera_depth_optical_frame'

        # publish clouds
        full_cloud = pc2.create_cloud_xyz32(header, full_points.tolist())
        self.full_pub.publish(full_cloud)

        if len(object_points) > 0:
            object_cloud = pc2.create_cloud_xyz32(header, object_points.tolist())
            self.object_pub.publish(object_cloud)

    def _lookup_transform(self, target_frame, source_frame, stamp_msg):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp_msg),
            )
        except TransformException as exc:
            warning_key = (target_frame, source_frame)
            if self.last_transform_warning != warning_key:
                self.get_logger().warning(
                    f"Waiting for transform {target_frame} <- {source_frame}: {exc}"
                )
                self.last_transform_warning = warning_key
            return None

    def _transform_points(self, points, transform):
        if len(points) == 0:
            return points

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        rotation = np.array([
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ], dtype=np.float32)
        translation = np.array([tx, ty, tz], dtype=np.float32)

        pts = np.asarray(points, dtype=np.float32)
        return pts @ rotation.T + translation

def main():
    rclpy.init()
    node = CloudBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
