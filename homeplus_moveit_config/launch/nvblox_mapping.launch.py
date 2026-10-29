"""Mapping viewer using the same nvblox/MoveIt path as manipulation.

Requires robot TF, calibrated camera streams, target segmentation and the hardware
bridge's settled status. Starts no motion executor or Arduino connection.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('homeplus_moveit_config'), 'launch', 'moveit.launch.py',
        ])), launch_arguments={
            'use_mapping': 'true', 'use_rviz': LaunchConfiguration('use_rviz'),
            'use_joint_gui': 'false', 'launch_robot_state_publisher': 'false',
            'run_arduino_reader': 'false', 'run_rebuild_map': 'false',
        }.items()),
    ])
