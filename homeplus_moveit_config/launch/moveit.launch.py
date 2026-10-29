#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def _launch_setup(context):
    use_mapping = LaunchConfiguration('use_octomap').perform(context).lower() in ('true', '1', 'yes', 'on')

    if LaunchConfiguration('run_rebuild_map').perform(context).lower() in ('true', '1', 'yes', 'on'):
        raise RuntimeError('run_rebuild_map is retired; the executor manages /nvblox/clear')

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

    # OMPL consumes nvblox's collision meshes through ApplyPlanningScene.
    # Do not load PointCloudOctomapUpdater: it would add a second obstacle map.
    move_group_params = [moveit_config.to_dict()]
    mapping_nodes = []
    if use_mapping:
        mapping_node = Node(
            package='homeplus_nvblox', executable='nvblox_mapper', name='nvblox_mapper',
            output='screen', parameters=[
                os.path.join(get_package_share_directory('homeplus_nvblox'), 'config', 'nvblox.yaml'),
                {'robot_description': robot_description,
                 'robot_description_semantic': robot_description_semantic},
            ],
        )
        mapping_nodes = [mapping_node, RegisterEventHandler(OnProcessExit(
            target_action=mapping_node,
            on_exit=[EmitEvent(event=Shutdown(reason='nvblox mapper exited'))],
        ))]

    # Create launch arguments
    auto_start_test = LaunchConfiguration('auto_start_test')
    run_arduino_reader = LaunchConfiguration('run_arduino_reader')

    return mapping_nodes + [
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
            cmd=['python3', os.path.join(homeplus_moveit_pkg, 'scripts', 'arduino_reader.py'), '--port', 'auto', '--baud', '115200'],
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
                    condition=IfCondition(LaunchConfiguration('use_rviz')),
                ),
            ]
        ),

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mapping',
            default_value='true',
            description='Enable nvblox collision reconstruction for MoveIt planning',
        ),
        DeclareLaunchArgument(
            'use_octomap', default_value=LaunchConfiguration('use_mapping'),
            description='Deprecated alias for use_mapping; mapping now uses nvblox',
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
            description='Deprecated: must remain false; nvblox rebuilding is managed by the executor',
        ),
        DeclareLaunchArgument(
            'launch_robot_state_publisher',
            default_value='true',
            description='If true, start robot_state_publisher here. Set false when another launch already owns it.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
