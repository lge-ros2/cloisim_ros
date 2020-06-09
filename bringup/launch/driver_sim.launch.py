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
from simdevice_bringup.common import get_modified_params_with_ns_and_remapping_list
from simdevice_bringup.common import find_robot_name
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')

    # Get the launch directory
    _pkg_name = "simdevice_bringup"
    launch_dir = os.path.join(get_package_share_directory(_pkg_name), 'launch')

    # define launch script name
    # launch script name
    #   ex) lidar_driver_sim.launch.py -> "lidar_driver_sim"
    _launch_list = [
        'lidar_driver_sim',
        'micom_driver_sim',
        'camera_driver_sim',
        # 'multi_camera_driver_sim'
    ]

    declare_launch_argument_rn = DeclareLaunchArgument(
        'robot_name',
        default_value=find_robot_name(),
        description='It is robot name. same as `node namspace`')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_launch_argument_rn)

    # Add the actions to launch all of sim driver
    for _item in _launch_list:

        included_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_dir, '/' + _item + '.launch.py']),
            launch_arguments={'robot_name': robot_name}.items())

        ld.add_action(included_launch)

    return ld
