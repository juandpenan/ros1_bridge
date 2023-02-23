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

import launch
import launch_ros
import lifecycle_msgs
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Get the launch directory
    ros1brdge_dir = get_package_share_directory('ros1_bridge')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    tf_static_1_to_2_node = LifecycleNode(
            name='tf_static_1_to_2',
            package='ros1_bridge',
            executable='tf_static_1_to_2',
            output='screen',
            parameters=[])

    tf_static_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(tf_static_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    tf_static_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(tf_static_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    tf_1_to_2_node = LifecycleNode(
            name='tf_1_to_2',
            package='ros1_bridge',
            executable='tf_1_to_2',
            output='screen',
            parameters=[])

    tf_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(tf_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    tf_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(tf_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    scan_1_to_2_node = LifecycleNode(
            name='scan_1_to_2',
            package='ros1_bridge',
            executable='scan_1_to_2',
            output='screen',
            parameters=[])

    scan_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(scan_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    scan_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(scan_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    odom_1_to_2_node = LifecycleNode(
            name='odom_1_to_2',
            package='ros1_bridge',
            executable='odom_1_to_2',
            output='screen',
            parameters=[])

    odom_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(odom_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    odom_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(odom_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    twist_2_to_1_node = LifecycleNode(
            name='twist_2_to_1',
            package='ros1_bridge',
            executable='twist_2_to_1',
            output='screen',
            parameters=[])

    twist_2_to_1_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(twist_2_to_1_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    twist_2_to_1_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(twist_2_to_1_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    pc2_1_to_2_node = LifecycleNode(
            name='pc2_1_to_2',
            package='ros1_bridge',
            executable='pc2_1_to_2',
            output='screen',
            parameters=[])

    pc2_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(pc2_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    pc2_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(pc2_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    imu_1_to_2_node = LifecycleNode(
            name='imu_1_to_2',
            package='ros1_bridge',
            executable='imu_1_to_2',
            output='screen',
            parameters=[])

    imu_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(imu_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    imu_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(imu_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    image_1_to_2_node = LifecycleNode(
            name='image_1_to_2',
            package='ros1_bridge',
            executable='image_1_to_2',
            output='screen',
            parameters=[])

    image_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(image_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    image_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(image_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    moveit_2_to_1_node = LifecycleNode(
            name='moveit_2_to_1',
            package='ros1_bridge',
            executable='moveit_2_to_1',
            output='screen',
            parameters=[])
    
    camera_info_1_to_2_node = LifecycleNode(
            name='camera_info_1_to_2',
            package='ros1_bridge',
            executable='camera_info_1_to_2',
            output='screen',
            parameters=[])

    camera_info_1_to_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(camera_info_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    camera_info_1_to_2_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(camera_info_1_to_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        tf_static_1_to_2_node, tf_1_to_2_node, scan_1_to_2_node,
        odom_1_to_2_node, twist_2_to_1_node, pc2_1_to_2_node,
        imu_1_to_2_node, image_1_to_2_node, camera_info_1_to_2_node,
        tf_static_1_to_2_configure, tf_1_to_2_configure, scan_1_to_2_configure,
        odom_1_to_2_configure, twist_2_to_1_configure, pc2_1_to_2_configure,
        imu_1_to_2_configure, image_1_to_2_configure, camera_info_1_to_2_configure,
        tf_static_1_to_2_activate, tf_1_to_2_activate, scan_1_to_2_activate,
        odom_1_to_2_activate, twist_2_to_1_activate, pc2_1_to_2_activate,
        imu_1_to_2_activate, image_1_to_2_activate, camera_info_1_to_2_activate,
        moveit_2_to_1_node
        ])
