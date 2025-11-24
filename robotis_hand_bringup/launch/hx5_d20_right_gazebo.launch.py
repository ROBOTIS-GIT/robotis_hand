#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Sungho Woo, Hyunwoo Nam

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

import xacro


def generate_launch_description():
    # Launch Arguments
    robotis_hand_description_path = os.path.join(
        get_package_share_directory('robotis_hand_description')
    )

    robotis_hand_bringup_path = os.path.join(
        get_package_share_directory('robotis_hand_bringup')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(robotis_hand_bringup_path, 'worlds'),
            ':' + str(Path(robotis_hand_description_path).parent.resolve()),
        ],
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty_world', description='Gz sim World'
        ),
        DeclareLaunchArgument(
            'model', default_value='hx5_d20_rev2', description='Robot model name'
        ),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])
        ],
    )

    model = LaunchConfiguration('model')

    xacro_file = PathJoinSubstitution([
        robotis_hand_description_path,
        'urdf',
        model,
        'hx5_d20_right.xacro'
    ])

    robot_desc_command = Command([
        'xacro',
        ' ',
        xacro_file,
        ' ',
        'use_sim:=true'
    ])

    robot_desc_content = ParameterValue(
        value=robot_desc_command,
        value_type=str
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc_content}
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_desc_command,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
            '-name',
            'robotis_hand',
            '-allow_renaming',
            'true',
            '-use_sim',
            'true',
        ],
    )

    # Controller spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_hand_controller'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    rviz_config_file = os.path.join(
        robotis_hand_description_path, 'rviz', 'robotis_hand.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[hand_controller_spawner],
            )
        ),
        bridge,
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        rviz,
    ])
