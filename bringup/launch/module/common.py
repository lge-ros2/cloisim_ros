#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import sys
import yaml
from tempfile import NamedTemporaryFile


def get_launcher_file_by_device_type(device_type):

    launcher_filename = {'MICOM': 'micom_driver_sim',
                         'LIDAR': 'lidar_driver_sim',
                         'LASER': 'lidar_driver_sim',
                         'CAMERA': 'camera_driver_sim',
                         'DEPTHCAMERA': 'depth_camera_driver_sim',
                         'MULTICAMERA': 'multi_camera_driver_sim',
                         'REALSENSE': 'realsense_driver_sim',
                         'GPS': 'gps_driver_sim',
                         'ELEVATOR': 'elevator_system_sim'}.get(device_type, None)

    return None if (launcher_filename is None) else launcher_filename + ".launch.py"


def get_robot_name_in_arg():

    _robot_name = ''

    # find ros param setting template
    for idx in range(4, len(sys.argv)):
        tmp_arg = sys.argv[idx]
        tmp_sp = tmp_arg.split(':=')

        # if argument is ros param template
        if len(tmp_sp) == 2:
            tmp_key = tmp_sp[0]
            tmp_value = tmp_sp[1]

            # check param name is robot_name
            if str(tmp_key) == 'robot_name' or str(tmp_key) == '_robot_name':
                # set namespace with robot_name
                _robot_name = tmp_value

    return _robot_name


def get_robot_name_in_env():

    _robot_name = ''

    # check environment param
    if 'ROBOT_NAME' in os.environ.keys():
        # if environment param name has ROBOT_NAME
        env_robot_name = os.environ['ROBOT_NAME']

        if env_robot_name != None:
            # set namespace with ROBOT_NAME
            _robot_name = env_robot_name

    return _robot_name


def find_robot_name():

    _robot_name = get_robot_name_in_arg()

    # if robot_name still not exist
    if len(_robot_name.strip()) == 0:
        _robot_name = get_robot_name_in_env()

    return _robot_name


def get_default_remapping_list():

    default_remapping_topic_list = ["/tf", "/tf_static"]

    result_remapping_list = set()

    # set topic remap list with prefix
    for remap_topic in default_remapping_topic_list:
        result_remapping_list.add((remap_topic, remap_topic.replace("/", "")))

    # print(result_remapping_list)
    return result_remapping_list


def _create_params_file_from_dict(params):
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
        param_file_path = h.name
        yaml.dump(params, h, default_flow_style=False)
        return param_file_path


def generate_temp_params_with_ns(_namespace, _node_name, port_maps):

    # capsule config with namespace
    config_dict = dict()
    config_dict[_namespace] = dict()
    config_dict[_namespace][_node_name] = dict()
    config_dict[_namespace][_node_name]['ros__parameters'] = dict()
    config_dict[_namespace][_node_name]['ros__parameters']['bridge'] = port_maps

    # print(config_dict)

    # create temp file for config load
    result_config_param = _create_params_file_from_dict(config_dict)
    # print(type(result_config_param))
    # print(result_config_param)

    return result_config_param
