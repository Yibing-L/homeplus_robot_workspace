#!/usr/bin/env python3
"""Launch cloud_builder, octomap_server and RViz configured for testing.

This launch file assumes you have:
- a depth camera publishing an aligned depth image to the topic configured below
- the package `octomap_server` installed
- the `homeplus_moveit_config` package built/installed (so this script or installed node is available)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('homeplus_moveit_config')

    octomap_params = os.path.join(pkg_share, 'config', 'octomap_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'moveit.rviz')

    cloud_builder_node = Node(
        package='homeplus_moveit_config',
        # the installed script filename
        executable='cloud_builder.py',
        name='cloud_builder',
        output='screen',
        parameters=[
            # adjust these if your camera publishes on different topics
            {'depth_topic': '/camera/camera/depth/image_rect_raw'},
            # Some camera drivers (like the RealSense node) publish camera_info under
            # '/camera/camera/color/camera_info' — adjust if your camera uses a different namespace
            {'camera_info_topic': '/camera/camera/depth/camera_info'},
            {'mask_topic': '/object_mask'}
        ]
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[octomap_params]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        cloud_builder_node,
        octomap_node,
        rviz_node,
    ])
