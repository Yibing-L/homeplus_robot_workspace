#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
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

    gesture_config_file_arg = DeclareLaunchArgument(
        "gesture_config_file",
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
            "enable_accel": "false",
            "enable_gyro": "false",
            "enable_motion": "false",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "enable_sync": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    gesture_node = Node(
        package="homeplus_vision",
        executable="gesture_recognizer.py",
        name="gesture_recognizer",
        parameters=[
            ParameterFile(LaunchConfiguration("gesture_config_file"), allow_substs=True),
            {
                "checkpoint_path": ParameterValue(
                    LaunchConfiguration("checkpoint_path"),
                    value_type=str,
                ),
                "label_map_path": ParameterValue(
                    LaunchConfiguration("label_map_path"),
                    value_type=str,
                ),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            checkpoint_path_arg,
            label_map_path_arg,
            gesture_config_file_arg,
            realsense_launch,
            gesture_node,
        ]
    )
