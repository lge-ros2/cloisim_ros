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
from cloisim_ros_bringup.common import get_package_name_by_device_type
from cloisim_ros_bringup.common import get_target_device_list
from cloisim_ros_bringup.common import generate_temp_params
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Get the launch directory
    _pkg_name = "cloisim_ros_bringup"
    launch_dir = os.path.join(get_package_share_directory(_pkg_name), 'launch')

    # Elevator
    model_name = "SeochoTower"
    elevator_list = get_target_device_list(model_name);

    for (device_type, nodes) in elevator_list.items():
        print(device_type)

        for (name, port_maps) in nodes.items():
            print("\t", name)

            package_name = get_package_name_by_device_type(device_type)
            print("\t > ", package_name)

            _config_params = generate_temp_params(name, model_name, port_maps)

            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_dir, '/cloisim_ros.launch.py']),
                    launch_arguments={'package_name':package_name, 'name': name, 'parameters': _config_params}.items())

            ld.add_action(included_launch)


    # World
    model_name = "World"
    world_list = get_target_device_list(model_name);

    for (device_type, nodes) in world_list.items():
        print(device_type)

        for (name, port_maps) in nodes.items():
            print("\t", name)

            package_name = get_package_name_by_device_type(device_type)
            print("\t > ", package_name)

            _config_params = generate_temp_params(name, model_name, port_maps)

            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_dir, '/cloisim_ros.launch.py']),
                    launch_arguments={'package_name':package_name, 'name': name, 'parameters': _config_params}.items())

            ld.add_action(included_launch)

    return ld