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
            'enable_color': 'true',
            'enable_depth': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30.0',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30.0',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
        }.items()
    )
    
    return LaunchDescription([
        realsense_launch,
    ])
