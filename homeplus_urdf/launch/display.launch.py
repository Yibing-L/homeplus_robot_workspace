from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('homeplus_urdf')
    urdf_path = os.path.join(package_dir, 'urdf', 'homeplus_urdf.urdf')

    # Verify URDF file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")

    # Load URDF file
    try:
        with open(urdf_path, 'r') as infp:
            robot_description_content = infp.read()
    except Exception as e:
        raise RuntimeError(f"Failed to read URDF file: {str(e)}")

    # Create default rviz config path
    rviz_config_path = os.path.join(package_dir, 'config', 'robot.rviz')
    if not os.path.exists(rviz_config_path):
        rviz_config_path = ""  # If config doesn't exist, let RViz use default settings

    return LaunchDescription([
        # Log the URDF path being used
        LogInfo(msg=['Loading URDF from: ', urdf_path]),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content,
                        'use_sim_time': False}],
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
            arguments=['-d', rviz_config_path] if rviz_config_path else [],
        )
    ])
