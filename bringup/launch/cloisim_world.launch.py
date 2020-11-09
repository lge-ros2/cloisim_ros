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
from sim_device_bringup.common import get_launcher_file_by_device_type
from sim_device_bringup.common import get_target_device_list
from sim_device_bringup.common import generate_temp_params
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get the launch directory
    _pkg_name = "sim_device_bringup"
    launch_dir = os.path.join(get_package_share_directory(_pkg_name), 'launch')

    included_launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/world_sim.launch.py']))

    sim_path = LaunchConfiguration('sim_path')
    world = LaunchConfiguration('world')

    included_launch_cloisim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/cloisim.launch.py']),
        launch_arguments={'sim_path': sim_path, 'world': world}.items())

    declare_launch_argument_sim_path = DeclareLaunchArgument(
        'sim_path',
        default_value='',
        description='path where the CLOiSim simulator located')

    declare_launch_argument_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='It is World file name. Please check environments before run. [CLOISIM_WORLD_PATH, CLOISIM_MODEL_PATH]')

    # Create environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add the actions to launch all nodes
    ld.add_action(declare_launch_argument_sim_path)
    ld.add_action(declare_launch_argument_world)

    ld.add_action(included_launch_cloisim)
    ld.add_action(included_launch_world)

    return ld