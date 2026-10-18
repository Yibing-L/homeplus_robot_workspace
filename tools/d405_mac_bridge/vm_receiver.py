#!/usr/bin/env python3
"""Republish the Mac-hosted D405 stream as realsense-ros-compatible topics in the VM.

Publishes (same names/frames as multi_camera.launch.py + realsense2_camera):
  /cameras/d405/color/image_raw                     rgb8   d405_color_optical_frame
  /cameras/d405/color/camera_info
  /cameras/d405/aligned_depth_to_color/image_raw    16UC1 (mm)  d405_color_optical_frame
  /cameras/d405/aligned_depth_to_color/camera_info
and static TF hand_camera_link -> d405_link -> d405_{depth,color}_frame -> *_optical_frame.

Color and aligned depth share one stamp: VM ROS time at receipt, so frames are never
ahead of the ROS clock.

    python3 vm_receiver.py --ros-args -p mac_host:=192.168.64.1 -p mac_port:=8766
"""

import json
import socket
import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

NS = '/cameras/d405'
CAM = 'd405'
# body (x fwd, y left, z up) -> optical (z fwd, x right, y down), as realsense-ros
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)


def recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError('sender closed connection')
        got += k
    return bytes(buf)


def recv_msg(sock):
    (hlen,) = struct.unpack('!I', recv_exact(sock, 4))
    return json.loads(recv_exact(sock, hlen))


def rot_to_quat(R):
    R = np.asarray(R, dtype=float)
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return (float(x), float(y), float(z), float(w))


def camera_info(intr, frame_id):
    m = CameraInfo()
    m.header.frame_id = frame_id
    m.width, m.height = intr['width'], intr['height']
    m.distortion_model = 'plumb_bob'
    coeffs = list(intr['coeffs'])[:5]
    m.d = [float(c) for c in coeffs] + [0.0] * (5 - len(coeffs))
    fx, fy, cx, cy = intr['fx'], intr['fy'], intr['ppx'], intr['ppy']
    m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return m


class D405MacBridge(Node):
    def __init__(self):
        super().__init__('d405_mac_bridge')
        self.host = self.declare_parameter('mac_host', '192.168.64.1').value
        self.port = int(self.declare_parameter('mac_port', 8766).value)
        self.mount_parent = self.declare_parameter('mount_parent', 'hand_camera_link').value
        self.color_frame = f'{CAM}_color_optical_frame'
        self.pub_color = self.create_publisher(Image, f'{NS}/color/image_raw', 5)
        self.pub_color_info = self.create_publisher(CameraInfo, f'{NS}/color/camera_info', 5)
        self.pub_depth = self.create_publisher(Image, f'{NS}/aligned_depth_to_color/image_raw', 5)
        self.pub_depth_info = self.create_publisher(
            CameraInfo, f'{NS}/aligned_depth_to_color/camera_info', 5)
        self.tf_static = StaticTransformBroadcaster(self)
        self.info = None

    def _tf(self, parent, child, xyz=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0)):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        return t

    def publish_static_tf(self, hello):
        ext = hello['depth_to_color']
        R = np.array(ext['rotation'], dtype=float).reshape(3, 3, order='F')
        t = np.array(ext['translation'], dtype=float)
        # Color optical frame origin/orientation expressed in the depth optical frame.
        R_dc = R.T
        p_dc = -R.T @ t
        # Express in body axes of the link (link == depth_frame), as realsense-ros does.
        opt_to_body = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        p_body = opt_to_body @ p_dc
        R_body = opt_to_body @ R_dc @ opt_to_body.T
        self.tf_static.sendTransform([
            self._tf(self.mount_parent, f'{CAM}_link'),
            self._tf(f'{CAM}_link', f'{CAM}_depth_frame'),
            self._tf(f'{CAM}_depth_frame', f'{CAM}_depth_optical_frame', q=OPTICAL_Q),
            self._tf(f'{CAM}_link', f'{CAM}_color_frame', xyz=tuple(p_body), q=rot_to_quat(R_body)),
            self._tf(f'{CAM}_color_frame', self.color_frame, q=OPTICAL_Q),
        ])
        self.get_logger().info(
            f'static TF published; depth->color offset (body) = {np.round(p_body, 5).tolist()} m')

    def run_once(self):
        sock = socket.create_connection((self.host, self.port), timeout=10.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(10.0)
        try:
            hello = recv_msg(sock)
            assert hello.get('type') == 'hello', hello
            self.get_logger().info(
                f"connected: {hello['device']} serial={hello['serial']} fw={hello['firmware']} "
                f"usb={hello['usb']} fps={hello['fps']} depth_scale={hello['depth_scale']}")
            self.publish_static_tf(hello)
            color_info = camera_info(hello['color_intrinsics'], self.color_frame)
            n, t_report = 0, time.time()
            while rclpy.ok():
                hdr = recv_msg(sock)
                cb = recv_exact(sock, hdr['color_len'])
                db = recv_exact(sock, hdr['depth_len'])
                stamp = self.get_clock().now().to_msg()
                w, h = hdr['w'], hdr['h']

                img = Image()
                img.header.stamp, img.header.frame_id = stamp, self.color_frame
                img.height, img.width, img.encoding, img.is_bigendian = h, w, 'rgb8', 0
                img.step, img.data = w * 3, cb

                dep = Image()
                dep.header.stamp, dep.header.frame_id = stamp, self.color_frame
                dep.height, dep.width, dep.encoding, dep.is_bigendian = h, w, '16UC1', 0
                dep.step, dep.data = w * 2, db

                color_info.header.stamp = stamp
                self.pub_color.publish(img)
                self.pub_color_info.publish(color_info)
                self.pub_depth.publish(dep)
                self.pub_depth_info.publish(color_info)

                n += 1
                if time.time() - t_report >= 10.0:
                    lag = time.time() - hdr['mac_time']
                    self.get_logger().info(
                        f'{n / (time.time() - t_report):.1f} fps, seq {hdr["seq"]}, '
                        f'mac->vm lag {lag * 1000:.0f} ms (clock offset included)')
                    n, t_report = 0, time.time()
        finally:
            sock.close()


def main():
    rclpy.init()
    node = D405MacBridge()
    try:
        while rclpy.ok():
            try:
                node.run_once()
            except (OSError, ConnectionError, AssertionError, ValueError) as e:
                node.get_logger().warn(f'stream to {node.host}:{node.port} lost ({e}); retrying in 2 s')
                time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
