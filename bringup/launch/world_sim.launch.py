#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import launch.actions
from ament_index_python.packages import get_package_share_directory
from simdevice_bringup.common import get_launcher_file_by_device_type
from simdevice_bringup.common import get_target_device_list
from simdevice_bringup.common import generate_temp_params
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Get the launch directory
    _pkg_name = "simdevice_bringup"
    launch_dir = os.path.join(get_package_share_directory(_pkg_name), 'launch')

    # Elevator
    model_name = "SeochoTower"
    elevator_list = get_target_device_list(model_name);

    for (device_type, nodes) in elevator_list.items():
        print(device_type)

        for (node_name, port_maps) in nodes.items():
            print("\t", node_name)

            launcher_filename = get_launcher_file_by_device_type(device_type)
            print("\t > ", launcher_filename)

            _config_params = generate_temp_params(node_name, model_name, port_maps)

            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_dir, '/', launcher_filename]),
                    launch_arguments={'model_name': model_name, 'node_name': node_name, 'parameters': _config_params}.items())

            ld.add_action(included_launch)


    # World
    model_name = "World"
    world_list = get_target_device_list(model_name);

    for (device_type, nodes) in world_list.items():
        print(device_type)

        for (node_name, port_maps) in nodes.items():
            print("\t", node_name)

            launcher_filename = get_launcher_file_by_device_type(device_type)
            print("\t > ", launcher_filename)

            _config_params = generate_temp_params(node_name, model_name, port_maps)

            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_dir, '/', launcher_filename]),
                launch_arguments={'node_name': node_name, 'parameters': _config_params}.items())

            ld.add_action(included_launch)

    # Create environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    return ld