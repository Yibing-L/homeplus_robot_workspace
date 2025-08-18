#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Declare launch arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='Size of ArUco markers in meters'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('homeplus_vision'),
            'config',
            'aruco_params.yaml'
        ]),
        description='Path to ArUco configuration file'
    )
    
    # ArUco detector node
    aruco_detector_node = Node(
        package='homeplus_vision',
        executable='aruco_detector.py',
        name='aruco_detector',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/camera/color/image_raw', '/camera/camera/color/image_raw'),
            ('/camera/color/camera_info', '/camera/camera/color/camera_info'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        marker_size_arg,
        config_file_arg,
        aruco_detector_node,
    ])
