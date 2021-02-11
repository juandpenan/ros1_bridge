# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    ros1brdge_dir = get_package_share_directory('ros1_bridge')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='ros1_bridge',
            executable='tf_static_1_to_2',
            output='screen',
            parameters=[]),

        Node(
            package='ros1_bridge',
            executable='tf_1_to_2',
            output='screen',
            parameters=[]),

        Node(
            package='ros1_bridge',
            executable='scan_1_to_2',
            output='screen',
            parameters=[]),

        Node(
            package='ros1_bridge',
            executable='odom_1_to_2',
            output='screen',
            parameters=[]),

        Node(
            package='ros1_bridge',
            executable='twist_2_to_1',
            output='screen',
            parameters=[]),

        Node(
            package='ros1_bridge',
            executable='imu_1_to_2',
            output='screen',
            parameters=[]),
        ])
