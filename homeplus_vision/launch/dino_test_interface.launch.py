#!/usr/bin/env python3

# Copyright 2026 Yibing Lu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Legacy filename for the production MolmoPoint node and interactive UI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _package_file(folder, name):
    return PathJoinSubstitution(
        [FindPackageShare('homeplus_vision'), folder, name]
    )


def generate_launch_description():
    caption = LaunchConfiguration('caption')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_file('launch', 'multi_camera.launch.py')
        ),
        launch_arguments={
            'enable_d405': 'true',
            'enable_d435i': 'false',
            'enable_d455': 'false',
            'd405_serial': LaunchConfiguration('d405_serial'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    detector = Node(
        package='homeplus_vision',
        executable='molmopoint_node.py',
        name='molmopoint',
        output='screen',
        parameters=[
            ParameterFile(
                _package_file('config', 'target_detection_params.yaml'),
                allow_substs=True,
            ),
            {
                'caption': ParameterValue(caption, value_type=str),
                'inference_rate': ParameterValue(
                    LaunchConfiguration('inference_rate'), value_type=float
                ),
                'publish_poses': ParameterValue(
                    LaunchConfiguration('publish_poses'), value_type=bool
                ),
                # Testing should not depend on the Arduino motion-status topic.
                'pause_during_motion': False,
                'world_frame': ParameterValue(
                    LaunchConfiguration('world_frame'), value_type=str
                ),
            },
        ],
    )

    interface = Node(
        package='homeplus_vision',
        executable='molmopoint_test_interface.py',
        name='molmopoint_test_interface',
        output='screen',
        parameters=[
            {
                'detector_node': '/molmopoint',
                'initial_caption': ParameterValue(caption, value_type=str),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('launch_camera', default_value='true'),
            DeclareLaunchArgument('d405_serial', default_value="''"),
            DeclareLaunchArgument('caption', default_value='the coffee cup'),
            DeclareLaunchArgument('inference_rate', default_value='0.5'),
            DeclareLaunchArgument('publish_poses', default_value='false'),
            DeclareLaunchArgument('world_frame', default_value='world'),
            camera,
            detector,
            interface,
        ]
    )
