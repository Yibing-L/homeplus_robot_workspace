"""Deprecated entry point; use nvblox_mapping.launch.py."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='cloud_to_octomap is retired; starting nvblox mapping. Start vision/TF/hardware state separately.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('homeplus_moveit_config'), 'launch', 'nvblox_mapping.launch.py',
        ]))),
    ])
