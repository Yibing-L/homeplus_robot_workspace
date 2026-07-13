#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'initial_reset': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_motion': 'false',
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
        }.items()
    )
    
    return LaunchDescription([
        realsense_launch,
    ])
