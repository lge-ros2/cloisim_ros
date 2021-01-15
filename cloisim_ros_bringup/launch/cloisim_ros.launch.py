#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import launch.actions
from cloisim_ros_bringup.common import get_default_remapping_list
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _package_name = LaunchConfiguration('package_name')
    _robot_name = LaunchConfiguration('robot_name')
    _name = LaunchConfiguration('name')
    _parameters = LaunchConfiguration('parameters')

    cloisim_ros_cmd = Node(
        package=_package_name,
        executable=_package_name,
        name=_name,
        namespace=_robot_name,
        output='screen',
        parameters=[_parameters],
        remappings=get_default_remapping_list())

    declare_launch_argument_rn = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='it is equal to namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

     # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_launch_argument_rn)
    ld.add_action(cloisim_ros_cmd)

    return ld
