#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="world",
        description="World frame used by grounding_dino_node pose output",
    )

    inference_rate_arg = DeclareLaunchArgument(
        "inference_rate",
        default_value="2.0",
        description="GroundingDINO inference loop frequency (Hz)",
    )

    task_id_arg = DeclareLaunchArgument(
        "task_id",
        default_value="1",
        description="GroundingDINO task preset id",
    )

    # Camera pose in world 
    camera_x_arg = DeclareLaunchArgument("camera_x", default_value="0.0")
    camera_y_arg = DeclareLaunchArgument("camera_y", default_value="0.0")
    camera_z_arg = DeclareLaunchArgument("camera_z", default_value="0.0")

    camera_qx_arg = DeclareLaunchArgument("camera_qx", default_value="0")
    camera_qy_arg = DeclareLaunchArgument("camera_qy", default_value="0")
    camera_qz_arg = DeclareLaunchArgument("camera_qz", default_value="0")
    camera_qw_arg = DeclareLaunchArgument("camera_qw", default_value="1")

    # Realsense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "initial_reset": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_accel": "false",
            "enable_gyro": "false",
            "enable_motion": "false",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "depth_module.infra_profile": "640x480x30",
            "enable_sync": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    world_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_camera_tf",
        arguments=[
            LaunchConfiguration("camera_x"),
            LaunchConfiguration("camera_y"),
            LaunchConfiguration("camera_z"),
            LaunchConfiguration("camera_qx"),
            LaunchConfiguration("camera_qy"),
            LaunchConfiguration("camera_qz"),
            LaunchConfiguration("camera_qw"),
            LaunchConfiguration("world_frame"),
            "hand_camera_link", 
        ],
    )

    # dino node launch
    gdino_node = Node(
        package="homeplus_vision",
        executable="grounding_dino_node.py",
        name="grounding_dino",
        output="screen",
        parameters=[
            {
                "image_topic": "/camera/camera/color/image_raw",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",
                "inference_rate": LaunchConfiguration("inference_rate"),
                "task_id": LaunchConfiguration("task_id"),
                "world_frame": LaunchConfiguration("world_frame"),
            }
        ],
    )

    return LaunchDescription(
        [
            world_frame_arg,
            inference_rate_arg,
            task_id_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            camera_qx_arg,
            camera_qy_arg,
            camera_qz_arg,
            camera_qw_arg,
            realsense_launch,
            world_to_camera_tf,
            gdino_node,
        ]
    )
