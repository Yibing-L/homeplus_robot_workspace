#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def _launch_setup(context):
    use_octomap = LaunchConfiguration('use_octomap').perform(context).lower() in (
        'true', '1', 'yes', 'on'
    )

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

    moveit_config = builder.to_moveit_configs()

    # OctoMap parameters — only passed to move_group when enabled
    move_group_params = [moveit_config.to_dict()]
    if use_octomap:
        octomap_sensors_path = os.path.join(
            homeplus_moveit_pkg, 'config', 'octomap_sensors.yaml'
        )
        with open(octomap_sensors_path, 'r') as f:
            move_group_params.append(yaml.safe_load(f))
        move_group_params.append({
            'octomap_frame': 'world',
            'octomap_resolution': 0.05,
        })

    # Create launch arguments
    auto_start_test = LaunchConfiguration('auto_start_test')
    run_arduino_reader = LaunchConfiguration('run_arduino_reader')
    run_rebuild_map = LaunchConfiguration('run_rebuild_map')

    return [
    # Robot State Publisher (skipped if pane 1 already owns it)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }],
            condition=IfCondition(LaunchConfiguration('launch_robot_state_publisher')),
        ),

        # Launch Arduino serial reader 
        ExecuteProcess(
            cmd=['python3', os.path.join(homeplus_moveit_pkg, 'scripts', 'arduino_reader.py'), '--port', 'auto', '--baud', '9600'],
            output='screen',
            condition=IfCondition(run_arduino_reader)
        ),

        # Launch map rebuilding node
        ExecuteProcess(
            cmd=['python3', os.path.join(homeplus_moveit_pkg, 'scripts', 'rebuild_map.py')],
            output='screen',
            condition=IfCondition(run_rebuild_map)
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
                    condition=IfCondition(LaunchConfiguration('use_rviz')),
                ),
            ]
        ),

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_octomap',
            default_value='false',
            description='Enable OctoMap-based collision avoidance in MoveIt planning',
        ),
        DeclareLaunchArgument(
            'auto_start_test',
            default_value='false',
            description='Whether to automatically start the IK test sequence',
        ),
        DeclareLaunchArgument(
            'run_arduino_reader',
            default_value='false',
            description='If true, start the Arduino serial reader node with automatic port detection',
        ),
        DeclareLaunchArgument(
            'use_joint_gui',
            default_value='true',
            description='Launch joint state publisher GUI (disable when using gesture pipeline)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the MoveIt MotionPlanning display',
        ),
        DeclareLaunchArgument(
            'run_rebuild_map',
            default_value='false',
            description='If true, start the map rebuilding node that listens for Arduino status and repopulates OctoMap',
        ),
        DeclareLaunchArgument(
            'launch_robot_state_publisher',
            default_value='true',
            description='If true, start robot_state_publisher here. Set false when another launch already owns it.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
