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

    use_octomap_arg = DeclareLaunchArgument(
        "use_octomap",
        default_value="true",
        description="Enable target-filtered OctoMap collision avoidance in MoveIt",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz through the MoveIt launch file",
    )

    launch_obstacle_cloud_arg = DeclareLaunchArgument(
        "launch_obstacle_cloud",
        default_value="false",
        description=(
            "Launch a legacy D405-only cloud node. Keep false when "
            "homeplus_vision vision_pipeline.launch.py is running"
        ),
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

    wait_for_hardware_ack_arg = DeclareLaunchArgument(
        "wait_for_hardware_ack",
        default_value="true",
        description=(
            "Wait for Arduino DONE after gripper and trajectory commands; "
            "set false only for visualization/bench tests"
        ),
    )

    continue_on_arduino_timeout_arg = DeclareLaunchArgument(
        "continue_on_arduino_timeout",
        default_value="false",
        description="Assume the final state after an Arduino timeout (unsafe on hardware)",
    )

    arduino_port_arg = DeclareLaunchArgument(
        "arduino_port",
        default_value="auto",
        description="Serial port for Arduino, or 'auto' to detect it",
    )

    arduino_baud_arg = DeclareLaunchArgument(
        "arduino_baud",
        default_value="115200",
        description="Arduino serial baud rate (must match Serial.begin in firmware)",
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
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_octomap": LaunchConfiguration("use_octomap"),
            # pane 1's state_publisher.launch.py already owns robot_state_publisher;
            # avoid spawning a duplicate that fights /tf and /robot_description.
            "launch_robot_state_publisher": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_moveit")),
    )

    occupancy_cloud_node = Node(
        package="homeplus_vision",
        executable="obstacle_cloud_node.py",
        name="d405_obstacle_cloud",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("homeplus_vision"), "config", "obstacle_clouds.yaml"]
            )
        ],
        condition=IfCondition(LaunchConfiguration("launch_obstacle_cloud")),
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
                # Clear before pose plans (pickup worked this way); steps that should
                # keep the map set clear_octomap_before_plan: false (place, 2026-10-30).
                "clear_octomap_before_pose_plan": LaunchConfiguration("use_octomap"),
                "wait_for_gripper_ack": LaunchConfiguration("wait_for_hardware_ack"),
                "wait_for_trajectory_ack": LaunchConfiguration("wait_for_hardware_ack"),
                "continue_on_arduino_timeout": LaunchConfiguration(
                    "continue_on_arduino_timeout"
                ),
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
            "--baud", LaunchConfiguration("arduino_baud"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_arduino")),
    )

    return LaunchDescription(
        [
            launch_moveit_arg,
            enable_moveit_planning_arg,
            use_octomap_arg,
            use_rviz_arg,
            launch_obstacle_cloud_arg,
            behavior_config_file_arg,
            behavior_map_path_arg,
            executor_config_file_arg,
            execution_map_path_arg,
            run_arduino_arg,
            wait_for_hardware_ack_arg,
            continue_on_arduino_timeout_arg,
            arduino_port_arg,
            arduino_baud_arg,
            moveit_launch,
            occupancy_cloud_node,
            behavior_node,
            executor_node,
            arduino_bridge_node,
        ]
    )
