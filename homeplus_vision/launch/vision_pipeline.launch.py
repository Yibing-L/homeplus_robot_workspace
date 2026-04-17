#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments (NO default values - config file is source of truth)
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='',
        description='Size of ArUco markers in meters (overrides config file)'
    )
    
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict_type',
        default_value='',
        description='ArUco dictionary type (overrides config file)'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='',
        description='Camera optical frame (overrides config file)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='',
        description='Robot base frame (overrides config file)'
    )
    
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
    
    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('homeplus_vision'),
        'config',
        'aruco_params.yaml'
    ])
    
    # ArUco detector node
    aruco_detector_node = Node(
        package='homeplus_vision',
        executable='aruco_detector.py',
        name='aruco_detector',
        parameters=[config_file],  # Config file is the single source of truth
        output='screen'
    )
    
    # ArUco pose publisher node
    aruco_pose_publisher_node = Node(
        package='homeplus_vision',
        executable='aruco_pose_publisher.py',
        name='aruco_pose_publisher',
        parameters=[config_file],  # Config file is the single source of truth
        output='screen'
    )
    
    # Static transform from camera_link to camera_color_optical_frame
    # Note: This may need adjustment based on your camera mounting
    camera_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf',
        arguments=[
            '0', '0', '0',  # translation
            '-0.5', '0.5', '-0.5', '0.5',  # rotation (quaternion)
            'camera_link',
            'camera_color_optical_frame'
        ]
    )
    
    # Static transform from base_link to camera_link
    # Note: Adjust these values based on your camera mounting position
    camera_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_tf',
        arguments=[
            '0.3', '0.0', '0.5',  # translation (30cm forward, 50cm up)
            '0', '0', '0', '1',   # rotation (no rotation)
            'base_link',
            'camera_link'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        marker_size_arg,
        aruco_dict_arg,
        camera_frame_arg,
        base_frame_arg,
        
        # Launch files and nodes
        realsense_launch,
        aruco_detector_node,
        aruco_pose_publisher_node,
        camera_optical_tf_node,
        camera_base_tf_node,
    ])
