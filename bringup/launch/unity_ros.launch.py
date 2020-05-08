#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # Get the launch directory
    _pkg_name = "simdevice_bringup"
    config_dir = os.path.join(get_package_share_directory(_pkg_name), 'config')

    config_params = os.path.join(config_dir, 'params.unity_ros.yaml')

    _node_name = 'unity_ros'
    start_unity_ros_init_cmd = Node(
        package=_node_name,
        node_executable=_node_name,
        node_name=_node_name,
        parameters=[config_params],
        output='screen')

    stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_envvar)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_unity_ros_init_cmd)

    return ld
