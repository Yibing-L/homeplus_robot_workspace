#!/usr/bin/env python3
"""Launch cloud_builder, octomap_server and RViz configured for testing.

This launch file assumes you have:
- a depth camera publishing an aligned depth image to the topic configured below
- the package `octomap_server` installed
- the `homeplus_moveit_config` package built/installed (so this script or installed node is available)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('homeplus_moveit_config')

    octomap_params = os.path.join(pkg_share, 'config', 'octomap_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'octomap.rviz')

    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    mask_topic = LaunchConfiguration('mask_topic')
    octomap_input_topic = LaunchConfiguration('octomap_input_topic')
    output_frame = LaunchConfiguration('output_frame')
    octomap_frame = LaunchConfiguration('octomap_frame')
    mask_dilation_pixels = LaunchConfiguration('mask_dilation_pixels')
    target_padding_m = LaunchConfiguration('target_padding_m')
    mask_timeout_sec = LaunchConfiguration('mask_timeout_sec')
    clear_octomap_service = LaunchConfiguration('clear_octomap_service')

    cloud_builder_node = Node(
        package='homeplus_moveit_config',
        # the installed script filename
        executable='cloud_builder.py',
        name='cloud_builder',
        output='screen',
        parameters=[
            {'depth_topic': depth_topic},
            {'camera_info_topic': camera_info_topic},
            {'mask_topic': mask_topic},
            {'output_frame': output_frame},
            {'mask_dilation_pixels': ParameterValue(mask_dilation_pixels, value_type=int)},
            {'target_padding_m': ParameterValue(target_padding_m, value_type=float)},
            {'mask_timeout_sec': ParameterValue(mask_timeout_sec, value_type=float)},
            {'clear_octomap_service': clear_octomap_service},
        ]
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[octomap_params, {'frame_id': octomap_frame}],
        remappings=[('cloud_in', octomap_input_topic)]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '-f', octomap_frame]
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('mask_topic', default_value='/object_mask'),
        DeclareLaunchArgument('output_frame', default_value=''),
        DeclareLaunchArgument('octomap_frame', default_value='hand_camera_depth_optical_frame'),
        DeclareLaunchArgument('octomap_input_topic', default_value='/obstacle_cloud'),
        DeclareLaunchArgument('mask_dilation_pixels', default_value='8'),
        DeclareLaunchArgument('target_padding_m', default_value='0.02'),
        DeclareLaunchArgument('mask_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument(
            'clear_octomap_service',
            default_value='/octomap_server/reset',
        ),
        cloud_builder_node,
        octomap_node,
        rviz_node,
    ])
