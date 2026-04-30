#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_moveit_arg = DeclareLaunchArgument(
        "launch_moveit",
        default_value="true",
        description="Launch the MoveIt stack before starting the gesture control nodes",
    )

    enable_moveit_planning_arg = DeclareLaunchArgument(
        "enable_moveit_planning",
        default_value="true",
        description="If true, the motion executor also sends MoveIt planning requests",
    )

    behavior_config_file_arg = DeclareLaunchArgument(
        "behavior_config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("homeplus_moveit_config"), "config", "gesture_behavior_params.yaml"]
        ),
        description="Path to gesture behavior parameter file",
    )

    behavior_map_path_arg = DeclareLaunchArgument(
        "behavior_map_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("homeplus_moveit_config"), "config", "gesture_behavior_map.yaml"]
        ),
        description="Path to gesture-to-command behavior map",
    )

    executor_config_file_arg = DeclareLaunchArgument(
        "executor_config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("homeplus_moveit_config"), "config", "gesture_executor_params.yaml"]
        ),
        description="Path to gesture motion executor parameter file",
    )

    execution_map_path_arg = DeclareLaunchArgument(
        "execution_map_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("homeplus_moveit_config"), "config", "gesture_execution_map.yaml"]
        ),
        description="Path to command-to-motion execution map",
    )

    run_arduino_arg = DeclareLaunchArgument(
        "run_arduino",
        default_value="true",
        description="Launch the Arduino serial bridge node",
    )

    arduino_port_arg = DeclareLaunchArgument(
        "arduino_port",
        default_value="/dev/ttyACM0",
        description="Serial port for Arduino",
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("homeplus_moveit_config"), "launch", "moveit.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_joint_gui": "false",
            # pane 1's state_publisher.launch.py already owns robot_state_publisher;
            # avoid spawning a duplicate that fights /tf and /robot_description.
            "launch_robot_state_publisher": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_moveit")),
    )

    behavior_node = Node(
        package="homeplus_moveit_config",
        executable="gesture_behavior.py",
        name="gesture_behavior",
        parameters=[
            LaunchConfiguration("behavior_config_file"),
            {
                "behavior_map_path": LaunchConfiguration("behavior_map_path"),
            },
        ],
        output="screen",
    )

    executor_node = Node(
        package="homeplus_moveit_config",
        executable="gesture_motion_executor.py",
        name="gesture_motion_executor",
        parameters=[
            LaunchConfiguration("executor_config_file"),
            {
                "execution_map_path": LaunchConfiguration("execution_map_path"),
                "enable_moveit_planning": LaunchConfiguration("enable_moveit_planning"),
            },
        ],
        output="screen",
    )

    arduino_bridge_node = Node(
        package="homeplus_moveit_config",
        executable="arduino_reader.py",
        name="arduino_bridge",
        arguments=[
            "--port", LaunchConfiguration("arduino_port"),
            "--baud", "9600",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_arduino")),
    )

    return LaunchDescription(
        [
            launch_moveit_arg,
            enable_moveit_planning_arg,
            behavior_config_file_arg,
            behavior_map_path_arg,
            executor_config_file_arg,
            execution_map_path_arg,
            run_arduino_arg,
            arduino_port_arg,
            moveit_launch,
            behavior_node,
            executor_node,
            arduino_bridge_node,
        ]
    )
