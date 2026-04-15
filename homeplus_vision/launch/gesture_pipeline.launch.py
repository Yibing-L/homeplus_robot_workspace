#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    checkpoint_path_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value="",
        description="Path to a train_xyz.py or train_with_angles.py .pt checkpoint",
    )

    label_map_path_arg = DeclareLaunchArgument(
        "label_map_path",
        default_value="",
        description="Optional JSON label map file for class names",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("homeplus_vision"), "config", "gesture_params.yaml"]
        ),
        description="Path to gesture recognizer config file",
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "color_width": "640",
            "color_height": "480",
            "color_fps": "30.0",
            "depth_width": "640",
            "depth_height": "480",
            "depth_fps": "30.0",
            "enable_sync": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    gesture_node = Node(
        package="homeplus_vision",
        executable="gesture_recognizer.py",
        name="gesture_recognizer",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "checkpoint_path": LaunchConfiguration("checkpoint_path"),
                "label_map_path": LaunchConfiguration("label_map_path"),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            checkpoint_path_arg,
            label_map_path_arg,
            config_file_arg,
            realsense_launch,
            gesture_node,
        ]
    )
