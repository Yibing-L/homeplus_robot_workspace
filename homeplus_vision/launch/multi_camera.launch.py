#!/usr/bin/env python3

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


"""Launch the three role-specific RealSense cameras used by HomePlus."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CAMERA_NAMESPACE = "cameras"


def _realsense(camera_name, device_type, serial_arg, enabled_arg):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            **({"config_file": PathJoinSubstitution([
                FindPackageShare("homeplus_vision"), "config", "d405_clock.yaml"
            ])} if camera_name == "d405" else {}),
            "camera_namespace": CAMERA_NAMESPACE,
            "camera_name": camera_name,
            "device_type": device_type,
            "serial_no": LaunchConfiguration(serial_arg),
            "initial_reset": LaunchConfiguration("initial_reset"),
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_accel": "false",
            "enable_gyro": "false",
            "enable_motion": "false",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "depth_module.infra_profile": "640x480x30",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "publish_tf": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration(enabled_arg)),
    )


def _mount_tf(name, parent, child, enabled_arg, xyz=(0.0, 0.0, 0.0), yaw=0.0):
    x, y, z = xyz
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--yaw", str(yaw),
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
        condition=IfCondition(LaunchConfiguration(enabled_arg)),
    )


def _side_mount_setup(context):
    side = LaunchConfiguration("side_mount").perform(context).strip().lower()
    if side not in ("left", "right"):
        raise RuntimeError("side_mount must be 'left' or 'right'")

    side_sign = 1.0 if side == "left" else -1.0
    y = side_sign * float(LaunchConfiguration("d455_side_offset_m").perform(context))
    yaw = side_sign * math.pi / 2.0
    x = float(LaunchConfiguration("d455_mount_x").perform(context))
    z = float(LaunchConfiguration("d455_mount_z").perform(context))

    return [
        LogInfo(msg=f"D455 configured on robot {side} side"),
        _mount_tf(
            "d455_mount_tf",
            "base_link",
            "d455_link",
            "enable_d455",
            xyz=(x, y, z),
            yaw=yaw,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_d405", default_value="true"),
            DeclareLaunchArgument("enable_d435i", default_value="true"),
            DeclareLaunchArgument("enable_d455", default_value="true"),
            DeclareLaunchArgument(
                "d405_serial",
                default_value="''",
                description="Optional D405 serial number",
            ),
            DeclareLaunchArgument(
                "d435i_serial",
                default_value="''",
                description="Optional D435i serial number",
            ),
            DeclareLaunchArgument(
                "d455_serial",
                default_value="''",
                description="Optional D455 serial number",
            ),
            DeclareLaunchArgument(
                "side_mount",
                default_value="left",
                description="D455 physical mounting side: left or right",
            ),
            DeclareLaunchArgument("d455_mount_x", default_value="0.0"),
            DeclareLaunchArgument("d455_side_offset_m", default_value="0.17907"),
            DeclareLaunchArgument("d455_mount_z", default_value="0.2794"),
            DeclareLaunchArgument(
                "initial_reset",
                default_value="false",
                description="Reset each RealSense before starting streams",
            ),
            LogInfo(
                msg="Camera roles: D405=target+OctoMap, D435i=OctoMap, D455=gesture+OctoMap"
            ),
            _realsense("d405", "d405", "d405_serial", "enable_d405"),
            _mount_tf(
                "d405_mount_tf",
                "hand_camera_link",
                "d405_link",
                "enable_d405",
            ),
            _realsense("d435i", "d435i", "d435i_serial", "enable_d435i"),
            _mount_tf(
                "d435i_mount_tf",
                "front_camera_link",
                "d435i_link",
                "enable_d435i",
            ),
            _realsense("d455", "d455", "d455_serial", "enable_d455"),
            OpaqueFunction(function=_side_mount_setup),
        ]
    )
