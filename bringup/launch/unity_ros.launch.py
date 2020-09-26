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
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _name = LaunchConfiguration('name')
    _parameters = LaunchConfiguration('parameters')

    _package_name = 'unity_ros'

    start_unity_ros_init_cmd = Node(
        package=_package_name,
        executable=_package_name,
        name=_name,
        parameters=[_parameters],
        output='screen')

    declare_launch_argument_nn = DeclareLaunchArgument(
        'name',
        default_value='unity_ros',
        description='it is node name')

    stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_envvar)

    ld.add_action(declare_launch_argument_nn)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_unity_ros_init_cmd)

    return ld
