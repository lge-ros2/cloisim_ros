#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import launch.actions
from simdevice_bringup.common import get_default_remapping_list
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _robot_name = LaunchConfiguration('robot_name')
    _node_name = LaunchConfiguration('node_name')
    _parameters = LaunchConfiguration('parameters')

    _package_name = 'micom_driver_sim'

    start_driver_cmd = Node(
        package=_package_name,
        node_executable=_package_name,
        node_name=_node_name,
        node_namespace=_robot_name,
        remappings=get_default_remapping_list(),
        parameters=[_parameters],
        output='screen')

    declare_launch_argument_nn = DeclareLaunchArgument(
        'node_name',
        default_value='micom',
        description='it is node name')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_launch_argument_nn)

    ld.add_action(start_driver_cmd)

    return ld
