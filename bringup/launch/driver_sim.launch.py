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
from simdevice_bringup.common import generate_temp_params_with_ns
from simdevice_bringup.common import find_robot_name
from simdevice_bringup.common import get_launcher_file_by_device_type
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import json
from websocket import create_connection

SIM_BIRDGE_IP="127.0.0.1"
SIM_BRIDGE_SERVICE_PORT=8080


def generate_launch_description():

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    driver_sim_robot_name = find_robot_name()

    ws = create_connection("ws://" + SIM_BIRDGE_IP + ":" + str(SIM_BRIDGE_SERVICE_PORT) + "/control")
    ws.send("{'command':'device_list', 'filter':'" + driver_sim_robot_name + "'}")
    result = ws.recv()
    # print("Received '%s'" % result)
    ws.close()

    device_list = json.loads(result)

    try:
        target_device_list = device_list["result"][driver_sim_robot_name]
    except Exception as inst:
        print(type(inst))       # the exception instance
        print(inst)
        print("Target robot name is valid: " + driver_sim_robot_name)
        return ld;

    # print(target_device_list)

    # Get the launch directory
    _pkg_name = "simdevice_bringup"
    launch_dir = os.path.join(get_package_share_directory(_pkg_name), 'launch')

    robot_name = LaunchConfiguration('robot_name')

    declare_launch_argument_rn = DeclareLaunchArgument(
        'robot_name',
        default_value=driver_sim_robot_name,
        description='It is robot name. same as `node namspace`')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    _robot_namespace = driver_sim_robot_name

    for (device_type, nodes) in target_device_list.items():
        print(device_type)

        for (node_name, port_maps) in nodes.items():
            print("\t", node_name)

            launcher_filename = get_launcher_file_by_device_type(device_type)
            print("\t > ", launcher_filename)

            if (launcher_filename is not None):

                _config_params = generate_temp_params_with_ns(_robot_namespace, node_name, port_maps)

                included_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [launch_dir, '/', launcher_filename]),
                        launch_arguments={'robot_name': robot_name, 'node_name': node_name, 'parameters': _config_params}.items())

                ld.add_action(included_launch)

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_launch_argument_rn)

    return ld
