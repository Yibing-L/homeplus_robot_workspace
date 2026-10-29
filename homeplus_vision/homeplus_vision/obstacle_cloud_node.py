# Copyright 2026 Yibing Lu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Build a reconstruction-ready obstacle cloud from one RGB-D camera."""

import json
from typing import Optional

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as point_cloud2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


class ObstacleCloudNode(Node):
    """Convert aligned depth into full, target, and target-filtered clouds."""

    def __init__(self) -> None:
        super().__init__("obstacle_cloud")

        self.declare_parameter("camera_role", "rgbd_camera")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("mask_topic", "")
        self.declare_parameter("fallback_frame", "")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("full_cloud_topic", "~/full")
        self.declare_parameter("object_cloud_topic", "~/target")
        self.declare_parameter("obstacle_cloud_topic", "/obstacle_cloud")
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("max_publish_rate_hz", 1.0)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.0)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("depth_consistency_threshold_m", 0.0)
        self.declare_parameter("depth_consistency_min_neighbors", 0)
        self.declare_parameter("filter_frame", "")
        self.declare_parameter("obstacle_min_z_m", -1000.0)
        self.declare_parameter("obstacle_max_z_m", 1000.0)
        self.declare_parameter("stats_interval_sec", 5.0)
        self.declare_parameter("pause_during_motion", False)
        self.declare_parameter("motion_status_topic", "/arduino/status")
        self.declare_parameter("task_id", 1)
        self.declare_parameter("mask_dilation_pixels", 8)
        self.declare_parameter("target_padding_m", 0.02)
        self.declare_parameter("button_task_id", 2)
        self.declare_parameter("button_target_padding_m", 0.08)
        self.declare_parameter("mask_timeout_sec", 1.0)
        self.declare_parameter("clear_octomap_on_mask", False)
        self.declare_parameter("clear_octomap_service", "/clear_octomap")
        self.declare_parameter("clear_octomap_min_interval_sec", 15.0)

        self.camera_role = str(self.get_parameter("camera_role").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value).strip()
        self.fallback_frame = str(self.get_parameter("fallback_frame").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.pixel_stride = max(1, int(self.get_parameter("pixel_stride").value))
        self.max_publish_rate_hz = max(
            0.0, float(self.get_parameter("max_publish_rate_hz").value)
        )
        self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)
        self.min_depth_m = max(0.0, float(self.get_parameter("min_depth_m").value))
        self.max_depth_m = max(0.0, float(self.get_parameter("max_depth_m").value))
        self.depth_consistency_threshold_m = max(
            0.0,
            float(self.get_parameter("depth_consistency_threshold_m").value),
        )
        self.depth_consistency_min_neighbors = max(
            0,
            min(4, int(self.get_parameter("depth_consistency_min_neighbors").value)),
        )
        self.filter_frame = str(self.get_parameter("filter_frame").value).strip()
        self.obstacle_min_z_m = float(
            self.get_parameter("obstacle_min_z_m").value
        )
        self.obstacle_max_z_m = float(
            self.get_parameter("obstacle_max_z_m").value
        )
        self.stats_interval_sec = max(
            0.0, float(self.get_parameter("stats_interval_sec").value)
        )
        self.pause_during_motion = bool(
            self.get_parameter("pause_during_motion").value
        )
        self.motion_status_topic = str(
            self.get_parameter("motion_status_topic").value
        ).strip()
        self.task_id = int(self.get_parameter("task_id").value)
        if self.obstacle_min_z_m > self.obstacle_max_z_m:
            raise ValueError("obstacle_min_z_m must not exceed obstacle_max_z_m")
        self.mask_dilation_pixels = max(
            0, int(self.get_parameter("mask_dilation_pixels").value)
        )
        default_target_padding_m = max(
            0.0, float(self.get_parameter("target_padding_m").value)
        )
        button_task_id = int(self.get_parameter("button_task_id").value)
        button_target_padding_m = max(
            0.0, float(self.get_parameter("button_target_padding_m").value)
        )
        self.target_padding_m = (
            button_target_padding_m
            if self.task_id == button_task_id
            else default_target_padding_m
        )
        self.mask_timeout_sec = max(
            0.0, float(self.get_parameter("mask_timeout_sec").value)
        )
        self.clear_octomap_on_mask = bool(
            self.get_parameter("clear_octomap_on_mask").value
        )
        self.clear_octomap_service = str(
            self.get_parameter("clear_octomap_service").value
        )
        self.clear_octomap_min_interval_sec = max(
            0.0,
            float(self.get_parameter("clear_octomap_min_interval_sec").value),
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_depth = None
        self.create_timer(0.02, self._process_pending_depth)

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.mask = None
        self.mask_received_at = None
        self.mask_stamp_ns = 0
        self.mask_was_active = False
        self.clear_request_pending = False
        self.last_octomap_clear_at = None
        self.last_transform_warning = None
        self.last_mask_shape_warning = None
        self.last_cloud_publish_at = None
        self.last_stats_at = None
        self.motion_active = self.pause_during_motion
        self.motion_finished_ns = 0
        self.ready_pub = self.create_publisher(String, "/vision/planning_cloud_ready", 10)

        self.full_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("full_cloud_topic").value), 10
        )
        self.object_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("object_cloud_topic").value), 10
        )
        self.obstacle_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("obstacle_cloud_topic").value), 10
        )

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        if self.mask_topic:
            self.create_subscription(
                Image,
                self.mask_topic,
                self.mask_callback,
                qos_profile_sensor_data,
            )
        if self.pause_during_motion and self.motion_status_topic:
            self.create_subscription(
                Bool,
                self.motion_status_topic,
                self.motion_status_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )

        self.clear_octomap_client = None
        if self.mask_topic and self.clear_octomap_on_mask:
            self.clear_octomap_client = self.create_client(
                Empty, self.clear_octomap_service
            )

        self.get_logger().info(
            f"Obstacle cloud ready for {self.camera_role}: "
            f"depth={self.depth_topic}, output={self.output_frame or 'source frame'}, "
            f"range={self.min_depth_m:.2f}..{self.max_depth_m:.2f} m, "
            f"filter_frame={self.filter_frame or 'output frame'}, "
            f"task={self.task_id}, target_padding={self.target_padding_m:.3f} m"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def mask_callback(self, msg: Image) -> None:
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.motion_active or stamp_ns <= self.motion_finished_ns:
            return
        self.mask_stamp_ns = stamp_ns
        self.mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.mask_received_at = self.get_clock().now()
        has_target = bool(np.any(self.mask))
        if has_target and not self.mask_was_active:
            self.mask_was_active = self._request_octomap_clear()
        elif not has_target:
            self.mask_was_active = False

    def motion_status_callback(self, msg: Bool) -> None:
        was_active = self.motion_active
        self.motion_active = not bool(msg.data)
        if self.motion_active and not was_active:
            # Pixel masks are tied to the camera pose at which they were
            # inferred. Never apply a pre-motion mask to the settled view.
            self.mask = None
            self.mask_received_at = None
            self.mask_was_active = False
            self.get_logger().info(
                "Pausing obstacle-cloud publication and invalidating the target "
                "mask during hardware motion"
            )
        elif was_active and not self.motion_active:
            self.motion_finished_ns = self.get_clock().now().nanoseconds
            self.get_logger().info(
                "Hardware settled; waiting for a fresh target mask before rebuilding the cloud"
            )
            # Executor owns map rebuilding after a valid settled observation.

    def depth_callback(self, msg: Image) -> None:
        # Retain a frame while joint-state TF catches up. Replacing it on every
        # camera callback would keep chasing timestamps newer than feedback.
        if self.pending_depth is None:
            self.pending_depth = (msg, self.get_clock().now().nanoseconds)

    def _process_pending_depth(self):
        if self.pending_depth is None:
            return
        msg, received_ns = self.pending_depth
        source = msg.header.frame_id or self.fallback_frame
        stamp = Time.from_msg(msg.header.stamp)
        frames = {self.output_frame, self.filter_frame}
        ready = bool(source) and all(
            not frame or frame == source or
            self.tf_buffer.can_transform(frame, source, stamp)
            for frame in frames)
        if not ready:
            if self.get_clock().now().nanoseconds - received_ns > 500_000_000:
                self.pending_depth = None
                self.get_logger().warning(
                    'Dropping depth frame: acquisition-time TF still missing after 0.5 s',
                    throttle_duration_sec=5.0)
            return
        self.pending_depth = None
        self._process_depth(msg)

    def _process_depth(self, msg: Image) -> None:
        if None in (self.fx, self.fy, self.cx, self.cy):
            return

        # The hand camera TF follows /joint_states. The Arduino bridge only has
        # a trustworthy final state, so integrating frames during motion would
        # stamp points at stale robot poses and leave ghost obstacles.
        if self.pause_during_motion and (self.motion_active or
                Time.from_msg(msg.header.stamp).nanoseconds <= self.motion_finished_ns):
            return

        now = self.get_clock().now()
        if self.max_publish_rate_hz > 0.0 and self.last_cloud_publish_at is not None:
            elapsed = (now - self.last_cloud_publish_at).nanoseconds / 1e9
            if elapsed < 1.0 / self.max_publish_rate_hz:
                return
        self.last_cloud_publish_at = now

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        source_frame = msg.header.frame_id or self.fallback_frame
        if not source_frame:
            self.get_logger().error("Depth image has no frame_id and no fallback_frame")
            return

        transform = None
        if self.output_frame and self.output_frame != source_frame:
            transform = self._lookup_transform(
                self.output_frame, source_frame, msg.header.stamp
            )
            if transform is None:
                return

        height, width = depth.shape
        mask = self._current_mask()
        if self.mask_topic and mask is None:
            return  # Do not reinsert the target while awaiting segmentation.
        if mask is not None and mask.shape != (height, width):
            import cv2

            shape_pair = (mask.shape, (height, width))
            if self.last_mask_shape_warning != shape_pair:
                self.get_logger().warning(
                    f"Resizing target mask {mask.shape} to depth {(height, width)}"
                )
                self.last_mask_shape_warning = shape_pair
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

        stride = self.pixel_stride
        depth_sample = depth[::stride, ::stride].astype(np.float32)
        if depth.dtype == np.uint16:
            depth_sample *= self.depth_unit_scale

        v_pixels, u_pixels = np.mgrid[0:height:stride, 0:width:stride].astype(
            np.float32
        )
        valid = np.isfinite(depth_sample) & (depth_sample > 0.0)
        if self.min_depth_m > 0.0:
            valid &= depth_sample >= self.min_depth_m
        if self.max_depth_m > 0.0:
            valid &= depth_sample <= self.max_depth_m

        stable_valid = self._depth_consistency_filter(depth_sample, valid)

        full_points = self._points_from_depth(
            depth_sample, u_pixels, v_pixels, valid
        )
        object_points = np.empty((0, 3), dtype=np.float32)
        obstacle_points = full_points

        if mask is not None:
            import cv2

            raw_mask = mask > 0
            object_selection = stable_valid & raw_mask[::stride, ::stride]
            object_points = self._points_from_depth(
                depth_sample, u_pixels, v_pixels, object_selection
            )

            exclusion_mask = raw_mask.astype(np.uint8)
            if self.mask_dilation_pixels:
                radius = self.mask_dilation_pixels
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1)
                )
                exclusion_mask = cv2.dilate(exclusion_mask, kernel)

            obstacle_selection = stable_valid & ~exclusion_mask[
                ::stride, ::stride
            ].astype(bool)
            obstacle_points = self._points_from_depth(
                depth_sample, u_pixels, v_pixels, obstacle_selection
            )
            obstacle_points = self._remove_target_box(
                obstacle_points, object_points
            )
        else:
            obstacle_points = self._points_from_depth(
                depth_sample, u_pixels, v_pixels, stable_valid
            )

        if transform is not None:
            full_points = self._transform_points(full_points, transform)
            object_points = self._transform_points(object_points, transform)
            obstacle_points = self._transform_points(obstacle_points, transform)

        frame_id = self.output_frame or source_frame
        obstacle_points, bounds_removed = self._filter_obstacle_bounds(
            obstacle_points, frame_id, msg.header.stamp
        )
        header = Header(stamp=msg.header.stamp, frame_id=frame_id)
        self.full_pub.publish(
            point_cloud2.create_cloud_xyz32(header, full_points.tolist())
        )
        self.object_pub.publish(
            point_cloud2.create_cloud_xyz32(header, object_points.tolist())
        )
        self.obstacle_pub.publish(
            point_cloud2.create_cloud_xyz32(header, obstacle_points.tolist())
        )
        if len(obstacle_points) > 0:
            self.ready_pub.publish(String(data=json.dumps({'stamp_ns': Time.from_msg(msg.header.stamp).nanoseconds,
                'masked': mask is not None, 'mask_stamp_ns': self.mask_stamp_ns})))
        self._log_cloud_stats(
            now,
            raw_count=int(np.count_nonzero(valid)),
            stable_count=int(np.count_nonzero(stable_valid)),
            object_count=len(object_points),
            obstacle_count=len(obstacle_points),
            bounds_removed=bounds_removed,
            mask_active=mask is not None,
            frame_id=frame_id,
        )

    def _current_mask(self):
        if self.mask is None or self.mask_received_at is None:
            return None
        if self.mask_timeout_sec == 0.0:
            return self.mask
        age = (self.get_clock().now().nanoseconds - self.mask_stamp_ns) / 1e9
        if age <= self.mask_timeout_sec:
            return self.mask
        self.mask_was_active = False
        return None

    def _points_from_depth(self, depth, u_pixels, v_pixels, selection):
        z = depth[selection]
        x = (u_pixels[selection] - self.cx) * z / self.fx
        y = (v_pixels[selection] - self.cy) * z / self.fy
        return np.column_stack((x, y, z)).astype(np.float32, copy=False)

    def _depth_consistency_filter(self, depth, valid):
        """Reject isolated depth samples while preserving continuous surfaces."""
        threshold = self.depth_consistency_threshold_m
        required = self.depth_consistency_min_neighbors
        if threshold <= 0.0 or required <= 0:
            return valid

        neighbor_count = np.zeros(depth.shape, dtype=np.uint8)
        for row_shift, col_shift in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            shifted_depth = np.roll(depth, (row_shift, col_shift), axis=(0, 1))
            shifted_valid = np.roll(valid, (row_shift, col_shift), axis=(0, 1))
            if row_shift < 0:
                shifted_valid[-1, :] = False
            elif row_shift > 0:
                shifted_valid[0, :] = False
            if col_shift < 0:
                shifted_valid[:, -1] = False
            elif col_shift > 0:
                shifted_valid[:, 0] = False
            neighbor_count += (
                valid
                & shifted_valid
                & (np.abs(depth - shifted_depth) <= threshold)
            )
        return valid & (neighbor_count >= required)

    def _filter_obstacle_bounds(self, obstacle_points, point_frame, stamp_msg):
        if not len(obstacle_points):
            return obstacle_points, 0

        filter_points = obstacle_points
        filter_frame = self.filter_frame or point_frame
        if filter_frame != point_frame:
            transform = self._lookup_transform(filter_frame, point_frame, stamp_msg)
            if transform is None:
                # Publishing an unfiltered cloud is less safe than skipping one
                # update when the filtering transform is temporarily absent.
                return np.empty((0, 3), dtype=np.float32), len(obstacle_points)
            filter_points = self._transform_points(obstacle_points, transform)

        keep = (
            (filter_points[:, 2] >= self.obstacle_min_z_m)
            & (filter_points[:, 2] <= self.obstacle_max_z_m)
        )
        removed = int(len(obstacle_points) - np.count_nonzero(keep))
        return obstacle_points[keep], removed

    def _log_cloud_stats(
        self,
        now,
        raw_count,
        stable_count,
        object_count,
        obstacle_count,
        bounds_removed,
        mask_active,
        frame_id,
    ):
        if self.stats_interval_sec <= 0.0:
            return
        if self.last_stats_at is not None:
            elapsed = (now - self.last_stats_at).nanoseconds / 1e9
            if elapsed < self.stats_interval_sec:
                return
        self.last_stats_at = now
        self.get_logger().info(
            f"CLOUD_STATS role={self.camera_role} frame={frame_id} "
            f"raw={raw_count} stable={stable_count} target={object_count} "
            f"obstacles={obstacle_count} bounds_removed={bounds_removed} "
            f"mask={'active' if mask_active else 'missing'}"
        )

    def _remove_target_box(self, obstacle_points, object_points):
        if not len(obstacle_points) or not len(object_points):
            return obstacle_points
        lower = object_points.min(axis=0) - self.target_padding_m
        upper = object_points.max(axis=0) + self.target_padding_m
        inside = np.all(
            (obstacle_points >= lower) & (obstacle_points <= upper), axis=1
        )
        return obstacle_points[~inside]

    def _lookup_transform(self, target_frame, source_frame, stamp_msg):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time.from_msg(stamp_msg)
            )
        except TransformException as exc:
            self.get_logger().warning(f'Acquisition-time cloud TF missing: {exc}', throttle_duration_sec=5.0)
            return None

    @staticmethod
    def _transform_points(points, transform):
        if not len(points):
            return points
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        matrix = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=np.float32,
        )
        offset = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float32
        )
        return np.asarray(points, dtype=np.float32) @ matrix.T + offset

    def _request_octomap_clear(self, force: bool = False) -> bool:
        if self.clear_octomap_client is None:
            return True
        if self.clear_request_pending:
            return True
        now = self.get_clock().now()
        if not force and self.last_octomap_clear_at is not None:
            elapsed = (now - self.last_octomap_clear_at).nanoseconds / 1e9
            if elapsed < self.clear_octomap_min_interval_sec:
                return True
        if not self.clear_octomap_client.service_is_ready():
            self.get_logger().warning(
                f"Waiting for {self.clear_octomap_service} before clearing OctoMap",
                throttle_duration_sec=5.0,
            )
            return False
        self.clear_request_pending = True
        future = self.clear_octomap_client.call_async(Empty.Request())
        future.add_done_callback(self._octomap_clear_done)
        return True

    def _octomap_clear_done(self, future) -> None:
        self.clear_request_pending = False
        try:
            future.result()
            self.last_octomap_clear_at = self.get_clock().now()
            self.get_logger().info("Cleared OctoMap before target filtering")
        except Exception as exc:
            self.mask_was_active = False
            self.get_logger().error(f"Failed to clear OctoMap: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
