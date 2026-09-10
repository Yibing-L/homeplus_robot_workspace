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


"""Launch role-specific perception nodes for already-running cameras."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _config_file(name):
    return PathJoinSubstitution([FindPackageShare("homeplus_vision"), "config", name])


def _cloud_node(camera_name):
    return Node(
        package="homeplus_vision",
        executable="obstacle_cloud_node.py",
        name=f"{camera_name}_obstacle_cloud",
        output="screen",
        parameters=[
            ParameterFile(
                LaunchConfiguration("obstacle_config_file"), allow_substs=True
            ),
            {
                "task_id": ParameterValue(
                    LaunchConfiguration("task_id"), value_type=int
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration(f"enable_{camera_name}_octomap")),
    )


def generate_launch_description():
    target_node = Node(
        package="homeplus_vision",
        executable="molmopoint_node.py",
        name="molmopoint",
        output="screen",
        parameters=[
            ParameterFile(
                LaunchConfiguration("target_config_file"), allow_substs=True
            ),
            {
                "task_id": ParameterValue(
                    LaunchConfiguration("task_id"), value_type=int
                ),
                "inference_rate": ParameterValue(
                    LaunchConfiguration("target_inference_rate"), value_type=float
                ),
                "world_frame": ParameterValue(
                    LaunchConfiguration("world_frame"), value_type=str
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration("enable_target_detection")),
    )

    gesture_node = Node(
        package="homeplus_vision",
        executable="gesture_recognizer.py",
        name="gesture_recognizer",
        output="screen",
        parameters=[
            ParameterFile(
                LaunchConfiguration("gesture_config_file"), allow_substs=True
            ),
            {
                "checkpoint_path": ParameterValue(
                    LaunchConfiguration("gesture_checkpoint"), value_type=str
                ),
                "label_map_path": ParameterValue(
                    LaunchConfiguration("gesture_label_map"), value_type=str
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration("enable_gesture_detection")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_target_detection", default_value="true"),
            DeclareLaunchArgument("enable_gesture_detection", default_value="false"),
            DeclareLaunchArgument("enable_d405_octomap", default_value="true"),
            DeclareLaunchArgument("enable_d435i_octomap", default_value="false"),
            DeclareLaunchArgument("enable_d455_octomap", default_value="false"),
            DeclareLaunchArgument("task_id", default_value="1"),
            DeclareLaunchArgument("target_inference_rate", default_value="0.5"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("gesture_checkpoint", default_value=""),
            DeclareLaunchArgument("gesture_label_map", default_value=""),
            DeclareLaunchArgument(
                "target_config_file",
                default_value=_config_file("target_detection_params.yaml"),
            ),
            DeclareLaunchArgument(
                "gesture_config_file",
                default_value=_config_file("gesture_params.yaml"),
            ),
            DeclareLaunchArgument(
                "obstacle_config_file",
                default_value=_config_file("obstacle_clouds.yaml"),
            ),
            target_node,
            gesture_node,
            _cloud_node("d405"),
            _cloud_node("d435i"),
            _cloud_node("d455"),
        ]
    )
