#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _singlemode = LaunchConfiguration('singlemode')

    cloisim_ros_cmd = Node(
        package="cloisim_ros_bringup",
        executable="cloisim_ros_bringup",
        output='screen',
        parameters=[{'singlemode': _singlemode}])

    declare_launch_argument = DeclareLaunchArgument(
        'singlemode',
        default_value='False',
        description='whether to use single mode')

    stdout_log_use_stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')

    stdout_log_buf_stream_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

     # Set environment variables
    ld.add_action(stdout_log_use_stdout_envvar)
    ld.add_action(stdout_log_buf_stream_envvar)
    ld.add_action(declare_launch_argument)
    ld.add_action(cloisim_ros_cmd)

    return ld
