from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    pkg = 'homeplus_urdf_description'

    xacro_file = PathJoinSubstitution([
        FindPackageShare(pkg),
        'urdf',
        'homeplus_urdf.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])

    use_gui = LaunchConfiguration('use_gui')
    publish_joint_states = LaunchConfiguration('publish_joint_states')

    actions = [
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Launch joint_state_publisher_gui when installed'
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='true',
            description='Publish default joint states so the arm/hand TF chain exists'
        ),
        LogInfo(msg=['Loading Xacro from: ', xacro_file]),
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
    ]

    try:
        get_package_share_directory('joint_state_publisher_gui')
        actions.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                condition=IfCondition(use_gui),
            )
        )
    except PackageNotFoundError:
        pass

    try:
        get_package_share_directory('joint_state_publisher')
        actions.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                condition=IfCondition(publish_joint_states),
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(msg='joint_state_publisher not installed; revolute-joint TF will be missing.')
        )

    return LaunchDescription(actions)
