#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Check for use_octomap flag early (before LaunchConfiguration is available)
    use_octomap = any('use_octomap:=true' in arg for arg in sys.argv)

    # Paths and configurations
    homeplus_urdf_pkg = get_package_share_directory('homeplus_urdf_description')
    homeplus_moveit_pkg = get_package_share_directory('homeplus_moveit_config')

    # Load URDF, SRDF and planning config
    xacro_path = os.path.join(homeplus_urdf_pkg, 'urdf', 'homeplus_urdf.xacro')
    robot_description = Command(['xacro', ' ', xacro_path])

    srdf_path = os.path.join(homeplus_moveit_pkg, 'config', 'homeplus.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # Get RViz config from homeplus_urdf package
    rviz_config_file = os.path.join(homeplus_moveit_pkg, 'config', 'moveit.rviz')

    # Load kinematics.yaml
    kinematics_yaml = os.path.join(homeplus_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_yaml, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)

    builder = (
        MoveItConfigsBuilder(
            "homeplus", package_name="homeplus_moveit_config"
        )
        .robot_description(file_path=xacro_path)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
    )

    # Only load OctoMap sensor plugin when use_octomap:=true
    if use_octomap:
        builder = builder.sensors(file_path="config/octomap_sensors.yaml")

    moveit_config = builder.to_moveit_configs()

    # OctoMap parameters — only passed to move_group when enabled
    move_group_params = [moveit_config.to_dict()]
    if use_octomap:
        move_group_params.append({
            'octomap_frame': 'world',
            'octomap_resolution': 0.05,
        })

    # Create launch arguments
    auto_start_test = LaunchConfiguration('auto_start_test')
    run_arduino_reader = LaunchConfiguration('run_arduino_reader')

    # Declare launch arguments
    declare_use_octomap = DeclareLaunchArgument(
        'use_octomap',
        default_value='false',
        description='Enable OctoMap-based collision avoidance in MoveIt planning'
    )
    declare_auto_start_test_cmd = DeclareLaunchArgument(
        'auto_start_test',
        default_value='false',
        description='Whether to automatically start the IK test sequence'
    )
    declare_run_arduino_reader = DeclareLaunchArgument(
        'run_arduino_reader',
        default_value='false',
        description='If true, start the Arduino serial reader node (reads /dev/ttyUSB0)'
    )
    declare_use_joint_gui = DeclareLaunchArgument(
        'use_joint_gui',
        default_value='true',
        description='Launch joint state publisher GUI (disable when using gesture pipeline)'
    )

    # Create and return launch description
    return LaunchDescription([
    # Launch Arguments
    declare_use_octomap,
    declare_auto_start_test_cmd,
    declare_run_arduino_reader,
    declare_use_joint_gui,

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

        # Launch Arduino serial reader 
        ExecuteProcess(
            cmd=['python3', os.path.join(homeplus_moveit_pkg, 'scripts', 'arduino_reader.py'), '--port', '/dev/ttyUSB0', '--baud', '9600'],
            output='screen',
            condition=IfCondition(run_arduino_reader)
        ),

        # Joint State Publisher GUI (disabled during gesture pipeline)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('use_joint_gui')),
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
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
