from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo


def generate_launch_description():
    pkg = 'homeplus_urdf_description'

    xacro_file = PathJoinSubstitution([
        FindPackageShare(pkg),
        'urdf',
        'homeplus_urdf.xacro'
    ])

    rviz_file = PathJoinSubstitution([
        FindPackageShare(pkg),
        'launch',
        'urdf.rviz'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])

    return LaunchDescription([
        LogInfo(msg=['Loading Xacro from: ', xacro_file]),
        LogInfo(msg=['Loading RViz config from: ', rviz_file]),

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

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        )
    ])