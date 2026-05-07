#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
# Author: Howon Kim

import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('robotis_hand_playground'),
        'config',
        'param.yaml',
    )
    with open(param_file, 'r', encoding='utf-8') as yaml_file:
        hand_side = next(line for line in yaml_file if 'hand_side:' in line)
        hand_side = hand_side.split(':', 1)[1].split('#', 1)[0].strip().strip('"\'')

    tactile_force_controller = Node(
        package='robotis_hand_playground',
        executable='tactile_force_controller',
        name='tactile_force_controller',
        output='screen',
        parameters=[param_file],
    )

    tactile_rviz_node = Node(
        package='robotis_hand_playground',
        executable='tactile_rviz',
        name='tactile_rviz',
        output='screen',
        parameters=[{
            'hand_side': hand_side,
        }],
    )

    return LaunchDescription([
        tactile_force_controller,
        tactile_rviz_node,
    ])
