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
from cloisim_ros_bringup.common import generate_temp_params_with_ns
from cloisim_ros_bringup.common import find_robot_name
from cloisim_ros_bringup.common import get_package_name_by_device_type
from cloisim_ros_bringup.common import get_target_device_list
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    robot_name = find_robot_name()

    target_device_list = get_target_device_list(robot_name)

    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory("cloisim_ros_bringup"), 'launch')

    robot_namespace = LaunchConfiguration('robot_name')

    for (device_type, nodes) in target_device_list.items():
        print(device_type)

        for (parts_name, port_maps) in nodes.items():
            print("\t", parts_name)

            package_name = get_package_name_by_device_type(device_type)
            print("\t > ", package_name)

            _config_params = generate_temp_params_with_ns(robot_name, parts_name, port_maps)

            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_dir, '/cloisim_ros.launch.py']),
                    launch_arguments={'package_name':package_name, 'robot_name': robot_namespace,
                                      'name': parts_name, 'parameters': _config_params}.items())

            ld.add_action(included_launch)

    print("\t")

    return ld
