#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_srvs.srv import Empty
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
        self.declare_parameter('mask_dilation_pixels', 8)
        self.declare_parameter('target_padding_m', 0.02)
        self.declare_parameter('mask_timeout_sec', 1.0)
        self.declare_parameter('clear_octomap_service', '/clear_octomap')
        self.declare_parameter('clear_octomap_min_interval_sec', 15.0)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.mask_dilation_pixels = max(
            0, int(self.get_parameter('mask_dilation_pixels').value)
        )
        self.target_padding_m = max(
            0.0, float(self.get_parameter('target_padding_m').value)
        )
        self.mask_timeout_sec = max(
            0.0, float(self.get_parameter('mask_timeout_sec').value)
        )
        self.clear_octomap_service = str(
            self.get_parameter('clear_octomap_service').value
        )
        self.clear_octomap_min_interval_sec = max(
            0.0, float(self.get_parameter('clear_octomap_min_interval_sec').value)
        )

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clear_octomap_client = self.create_client(
            Empty, self.clear_octomap_service
        )

        self.mask = None
        self.mask_received_at = None
        self.mask_was_active = False
        self.clear_request_pending = False
        self.last_octomap_clear_at = None
        self.last_mask_shape_warning = None
        self.last_transform_warning = None

        self.camera_info_received = False

        # Publishers (IMPORTANT FOR MOVEIT)
        self.full_pub = self.create_publisher(PointCloud2, '/full_cloud', 10)
        self.object_pub = self.create_publisher(PointCloud2, '/object_cloud', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacle_cloud', 10)

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
        self.mask_received_at = self.get_clock().now()
        has_target = bool(np.any(self.mask))
        if has_target and not self.mask_was_active:
            self.mask_was_active = self._request_octomap_clear()
        elif not has_target:
            self.mask_was_active = False

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
        mask_for_depth = self._current_mask()
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

        # SAM2 mask (optional). The target is published separately and removed
        # conservatively from the cloud used for collision avoidance.
        object_points = np.empty((0, 3), dtype=np.float32)
        obstacle_points = full_points
        if mask_for_depth is not None:
            import cv2

            raw_mask = mask_for_depth > 0
            obj_mask = valid & raw_mask[::step, ::step]
            if np.any(obj_mask):
                z_obj = z[obj_mask]
                u_obj = u_indices[obj_mask]
                v_obj = v_indices[obj_mask]
                x_obj = (u_obj - self.cx) * z_obj / self.fx
                y_obj = (v_obj - self.cy) * z_obj / self.fy
                object_points = np.column_stack([x_obj, y_obj, z_obj])

            exclusion_mask = raw_mask.astype(np.uint8)
            if self.mask_dilation_pixels > 0:
                radius = self.mask_dilation_pixels
                kernel_size = 2 * radius + 1
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
                )
                exclusion_mask = cv2.dilate(exclusion_mask, kernel)

            obstacle_valid = valid & ~exclusion_mask[::step, ::step].astype(bool)
            obstacle_points = self._points_from_depth(
                z, u_indices, v_indices, obstacle_valid
            )

            # Remove residual target-edge points inside a padded 3D target box.
            # This complements image-space dilation without opening a large,
            # unconstrained hole in the collision cloud.
            if len(object_points) > 0 and len(obstacle_points) > 0:
                lower = object_points.min(axis=0) - self.target_padding_m
                upper = object_points.max(axis=0) + self.target_padding_m
                inside_target_box = np.all(
                    (obstacle_points >= lower) & (obstacle_points <= upper),
                    axis=1,
                )
                obstacle_points = obstacle_points[~inside_target_box]

        if transform is not None:
            full_points = self._transform_points(full_points, transform)
            if len(object_points) > 0:
                object_points = self._transform_points(object_points, transform)
            obstacle_points = self._transform_points(obstacle_points, transform)

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        if self.output_frame:
            header.frame_id = self.output_frame
        elif not header.frame_id:
            header.frame_id = 'hand_camera_depth_optical_frame'

        # publish clouds
        full_cloud = pc2.create_cloud_xyz32(header, full_points.tolist())
        self.full_pub.publish(full_cloud)

        object_cloud = pc2.create_cloud_xyz32(header, object_points.tolist())
        self.object_pub.publish(object_cloud)

        obstacle_cloud = pc2.create_cloud_xyz32(header, obstacle_points.tolist())
        self.obstacle_pub.publish(obstacle_cloud)

    def _current_mask(self):
        if self.mask is None or self.mask_received_at is None:
            return None
        if self.mask_timeout_sec == 0.0:
            return self.mask
        age = (self.get_clock().now() - self.mask_received_at).nanoseconds / 1e9
        if age <= self.mask_timeout_sec:
            return self.mask
        self.mask_was_active = False
        return None

    def _request_octomap_clear(self):
        if self.clear_request_pending:
            return True
        now = self.get_clock().now()
        if self.last_octomap_clear_at is not None:
            elapsed = (now - self.last_octomap_clear_at).nanoseconds / 1e9
            if elapsed < self.clear_octomap_min_interval_sec:
                return True
        if not self.clear_octomap_client.service_is_ready():
            self.get_logger().warning(
                f"Waiting for {self.clear_octomap_service} before clearing "
                "unfiltered target voxels"
            )
            return False
        self.clear_request_pending = True
        future = self.clear_octomap_client.call_async(Empty.Request())
        future.add_done_callback(self._octomap_clear_done)
        return True

    def _octomap_clear_done(self, future):
        self.clear_request_pending = False
        try:
            future.result()
            self.last_octomap_clear_at = self.get_clock().now()
            self.get_logger().info(
                "Cleared OctoMap as target-mask filtering became active"
            )
        except Exception as exc:
            self.mask_was_active = False
            self.get_logger().error(f"Failed to clear OctoMap: {exc}")

    def _points_from_depth(self, z, u_indices, v_indices, selection):
        z_selected = z[selection]
        u_selected = u_indices[selection]
        v_selected = v_indices[selection]
        x_selected = (u_selected - self.cx) * z_selected / self.fx
        y_selected = (v_selected - self.cy) * z_selected / self.fy
        return np.column_stack([x_selected, y_selected, z_selected])

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
