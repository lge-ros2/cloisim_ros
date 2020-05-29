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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _robot_name = LaunchConfiguration('robot_name')

    # Get the launch directory
    _pkg_name = "simdevice_bringup"

    _config_dir = os.path.join(get_package_share_directory(_pkg_name), 'config')
    config_params = os.path.join(_config_dir, 'params.multi_camera_driver.yaml')

    _package_name = 'multi_camera_driver_sim'
    _node_name = 'multi_camera_driver'

    # modify config param with namespace
    (_config_params, _remapping_list) = get_modified_params_with_ns_and_remapping_list(
        config_params, _node_name)

    start_multi_camera_driver_sim_cmd = Node(
        package=_package_name,
        node_executable=_package_name,
        node_name=_node_name,
        node_namespace=_robot_name,
        remappings=_remapping_list,
        parameters=[_config_params],
        output='screen')


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

    ld.add_action(start_multi_camera_driver_sim_cmd)

    return ld
