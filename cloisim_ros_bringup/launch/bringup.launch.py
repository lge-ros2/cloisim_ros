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
    _target_model = LaunchConfiguration('target_model')
    _target_parts = LaunchConfiguration('target_parts')

    cloisim_ros_cmd = Node(
        package="cloisim_ros_bringup",
        executable="cloisim_ros_bringup",
        output='screen',
        parameters=[{'singlemode': _singlemode, 'target_model': _target_model, 'target_parts': _target_parts}])

    declare_launch_argument_sm = DeclareLaunchArgument(
        'singlemode',
        default_value='False',
        description='whether to use single mode')

    declare_launch_argument_tm = DeclareLaunchArgument(
        'target_model',
        default_value='',
        description='specify the target model you want')

    declare_launch_argument_tp = DeclareLaunchArgument(
        'target_parts',
        default_value='',
        description='specify the target parts you want')

    stdout_log_use_stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')

    stdout_log_buf_stream_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

     # Set environment variables
    ld.add_action(stdout_log_use_stdout_envvar)
    ld.add_action(stdout_log_buf_stream_envvar)
    ld.add_action(declare_launch_argument_sm)
    ld.add_action(declare_launch_argument_tm)
    ld.add_action(declare_launch_argument_tp)
    ld.add_action(cloisim_ros_cmd)

    return ld
