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


"""Full HomePlus three-camera vision pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _vision_launch(name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("homeplus_vision"), "launch", name])
    )


def generate_launch_description():
    cameras = IncludeLaunchDescription(
        _vision_launch("multi_camera.launch.py"),
        launch_arguments={
            "enable_d405": LaunchConfiguration("enable_d405"),
            "enable_d435i": LaunchConfiguration("enable_d435i"),
            "enable_d455": LaunchConfiguration("enable_d455"),
            "d405_serial": LaunchConfiguration("d405_serial"),
            "d435i_serial": LaunchConfiguration("d435i_serial"),
            "d455_serial": LaunchConfiguration("d455_serial"),
            "side_mount": LaunchConfiguration("side_mount"),
            "initial_reset": LaunchConfiguration("initial_reset"),
        }.items(),
    )

    perception = IncludeLaunchDescription(
        _vision_launch("perception.launch.py"),
        launch_arguments={
            "enable_target_detection": LaunchConfiguration(
                "enable_target_detection"
            ),
            "enable_gesture_detection": LaunchConfiguration(
                "enable_gesture_detection"
            ),
            "enable_d405_octomap": LaunchConfiguration("enable_d405_octomap"),
            "enable_d435i_octomap": LaunchConfiguration("enable_d435i_octomap"),
            "enable_d455_octomap": LaunchConfiguration("enable_d455_octomap"),
            "task_id": LaunchConfiguration("task_id"),
            "target_inference_rate": LaunchConfiguration("target_inference_rate"),
            "world_frame": LaunchConfiguration("world_frame"),
            "gesture_checkpoint": LaunchConfiguration("gesture_checkpoint"),
            "gesture_label_map": LaunchConfiguration("gesture_label_map"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_d405", default_value="true"),
            DeclareLaunchArgument("enable_d435i", default_value="true"),
            DeclareLaunchArgument("enable_d455", default_value="true"),
            DeclareLaunchArgument("d405_serial", default_value="''"),
            DeclareLaunchArgument("d435i_serial", default_value="''"),
            DeclareLaunchArgument("d455_serial", default_value="''"),
            DeclareLaunchArgument("side_mount", default_value="left"),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("enable_target_detection", default_value="true"),
            DeclareLaunchArgument("enable_gesture_detection", default_value="false"),
            DeclareLaunchArgument("enable_d405_octomap", default_value="true"),
            # Only D405 currently receives the detected-target mask. Feeding
            # either unmasked base camera into the same map makes the cup an
            # obstacle again and can make every grasp goal collide.
            DeclareLaunchArgument("enable_d435i_octomap", default_value="false"),
            DeclareLaunchArgument("enable_d455_octomap", default_value="false"),
            DeclareLaunchArgument("task_id", default_value="1"),
            DeclareLaunchArgument("target_inference_rate", default_value="0.5"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("gesture_checkpoint", default_value=""),
            DeclareLaunchArgument("gesture_label_map", default_value=""),
            cameras,
            perception,
        ]
    )
