#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths and configurations
    homeplus_urdf_pkg = get_package_share_directory('homeplus_urdf')
    homeplus_moveit_pkg = get_package_share_directory('homeplus_moveit_config')

    # Load URDF, SRDF and planning config
    urdf_path = os.path.join(homeplus_urdf_pkg, 'urdf', 'homeplus_urdf.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    srdf_path = os.path.join(homeplus_moveit_pkg, 'config', 'homeplus.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # Get RViz config from homeplus_urdf package
    rviz_config_file = os.path.join(homeplus_moveit_pkg, 'config', 'moveit.rviz')

    # Load kinematics.yaml
    kinematics_yaml = os.path.join(homeplus_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_yaml, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)

    # Load OMPL Planning Pipeline
    ompl_planning_yaml = os.path.join(homeplus_moveit_pkg, 'config', 'ompl_planning.yaml')
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
            'enforce_bounds': True,
            'enforce_joint_model_state_space': True,
        }
    }
    if os.path.exists(ompl_planning_yaml):
        with open(ompl_planning_yaml, 'r') as f:
            ompl_planning_pipeline_config['ompl'] = yaml.safe_load(f)

    # MoveIt Controllers
    moveit_simple_controllers = {
        'moveit_simple_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'controller_names': ['base_arm_controller', 'arm_controller', 'gripper_controller'],
        'base_arm_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'default': True,
            'joints': ['base_x', 'base_y', 'base_theta', 'JointFrame', 'JointTelescope', 'JointArm1', 'JointArm2', 'JointWrist', 'JointHand'],
        },
        'arm_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'default': False,
            'joints': ['JointArm1', 'JointArm2', 'JointHand', 'JointTelescope', 'JointWrist'],
        },
        'gripper_controller': {
            'type': 'GripperCommand',
            'joints': ['JointFingerL', 'JointFingerR'],
            'action_ns': 'gripper_cmd',
            'default': False,
        },
    }

    # Create launch arguments
    auto_start_test = LaunchConfiguration('auto_start_test')
    
    # Declare launch arguments
    declare_auto_start_test_cmd = DeclareLaunchArgument(
        'auto_start_test',
        default_value='false',
        description='Whether to automatically start the IK test sequence'
    )

    # Create and return launch description
    return LaunchDescription([
        # Launch Arguments
        declare_auto_start_test_cmd,

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

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # Move Group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': robot_description_kinematics,
                'robot_description_planning': ompl_planning_pipeline_config,
                'moveit_controller_manager': moveit_simple_controllers['moveit_controller_manager'],
                'moveit_simple_controller_manager': moveit_simple_controllers,
                'use_sim_time': False
            }],
        ),

        # RViz
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

        # MoveIt Controller Node
        Node(
            package='homeplus_moveit_config',
            executable='moveit_controller.py',
            name='homeplus_moveit_controller',
            output='screen',
        ),

        # IK Test Node
        Node(
            package='homeplus_moveit_config',
            executable='ik.py',
            name='ik_test_node',
            output='screen',
        ),
    ])
