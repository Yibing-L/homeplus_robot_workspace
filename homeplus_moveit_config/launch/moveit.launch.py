#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths and configurations
    homeplus_urdf_pkg = get_package_share_directory('homeplus_urdf')
    homeplus_moveit_pkg = get_package_share_directory('homeplus_moveit_config')

    # Load URDF, SRDF and planning config
    urdf_path = os.path.join(homeplus_urdf_pkg, 'urdf', 'homeplus_urdf.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    srdf_path = os.path.join(homeplus_moveit_pkg, 'config', 'homeplus.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # Get RViz config from homeplus_urdf package
    rviz_config_file = os.path.join(homeplus_moveit_pkg, 'config', 'moveit.rviz')

    # Load kinematics.yaml
    kinematics_yaml = os.path.join(homeplus_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_yaml, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)

    moveit_config = (
        MoveItConfigsBuilder(
            "homeplus", package_name="homeplus_moveit_config"  # Update to your robot/group/package
        )
        .robot_description(file_path=urdf_path)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Create launch arguments
    auto_start_test = LaunchConfiguration('auto_start_test')
    
    # Declare launch arguments
    declare_auto_start_test_cmd = DeclareLaunchArgument(
        'auto_start_test',
        default_value='false',
        description='Whether to automatically start the IK test sequence'
    )

    # Create and return launch description
    return LaunchDescription([
        # Launch Arguments
        declare_auto_start_test_cmd,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }],
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),

        # RViz (delayed to start after move_group is established)
        TimerAction(
            period=3.0,  # Wait 3 seconds
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_file],
                    parameters=[{
                        'robot_description': robot_description,
                        'robot_description_semantic': robot_description_semantic,
                        'robot_description_kinematics': robot_description_kinematics,
                        'use_sim_time': False
                    }],
                ),
            ]
        ),

    ])
