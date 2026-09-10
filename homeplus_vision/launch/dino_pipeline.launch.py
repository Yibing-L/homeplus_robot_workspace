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


"""Legacy filename for the D405 MolmoPoint pipeline and optional obstacle cloud."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _source(name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("homeplus_vision"), "launch", name])
    )


def generate_launch_description():
    camera = IncludeLaunchDescription(
        _source("multi_camera.launch.py"),
        launch_arguments={
            "enable_d405": "true",
            "enable_d435i": "false",
            "enable_d455": "false",
            "d405_serial": LaunchConfiguration("d405_serial"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_camera")),
    )
    perception = IncludeLaunchDescription(
        _source("perception.launch.py"),
        launch_arguments={
            "enable_target_detection": "true",
            "enable_gesture_detection": "false",
            "enable_d405_octomap": LaunchConfiguration("enable_octomap"),
            "enable_d435i_octomap": "false",
            "enable_d455_octomap": "false",
            "task_id": LaunchConfiguration("task_id"),
            "target_inference_rate": LaunchConfiguration("inference_rate"),
            "world_frame": LaunchConfiguration("world_frame"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("d405_serial", default_value="''"),
            DeclareLaunchArgument("enable_octomap", default_value="false"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("inference_rate", default_value="0.5"),
            DeclareLaunchArgument("task_id", default_value="1"),
            camera,
            perception,
        ]
    )
