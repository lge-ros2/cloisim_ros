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
import json
import time
from tempfile import NamedTemporaryFile
from websocket import create_connection

SIM_BIRDGE_IP="127.0.0.1"
SIM_BRIDGE_SERVICE_PORT=8080

def get_package_name_by_device_type(device_type):

    package_name = {'MICOM': 'cloisim_ros_micom',
                    'LIDAR': 'cloisim_ros_lidar',
                    'LASER': 'cloisim_ros_lidar',
                    'CAMERA': 'cloisim_ros_camera',
                    'DEPTHCAMERA': 'cloisim_ros_depthcamera',
                    'MULTICAMERA': 'cloisim_ros_multicamera',
                    'REALSENSE': 'cloisim_ros_realsense',
                    'GPS': 'cloisim_ros_gps',
                    'ELEVATOR': 'cloisim_ros_elevatorsystem',
                    'WORLD': 'cloisim_ros_world'}.get(device_type, None)

    return None if (package_name is None) else package_name


def _get_robot_name_in_arg():

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

    _robot_name = _get_robot_name_in_arg()

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
    with NamedTemporaryFile(mode='w', prefix='cloisim_ros_launch_params_', delete=False) as h:
        param_file_path = h.name
        yaml.dump(params, h, default_flow_style=False)
        return param_file_path


def generate_temp_params(_name, model_name, port_maps):

    # capsule config with namespace
    config_dict = dict()
    config_dict[_name] = dict()
    config_dict[_name]['ros__parameters'] = dict()
    config_dict[_name]['ros__parameters']['model'] = model_name
    config_dict[_name]['ros__parameters']['bridge'] = port_maps

    # print(config_dict)

    # create temp file for config load
    result_config_param = _create_params_file_from_dict(config_dict)
    # print(type(result_config_param))
    # print(result_config_param)

    return result_config_param

def generate_temp_params_with_ns(_namespace, _name, port_maps):

    # capsule config with namespace
    config_dict = dict()
    config_dict[_namespace] = dict()
    config_dict[_namespace][_name] = dict()
    config_dict[_namespace][_name]['ros__parameters'] = dict()
    config_dict[_namespace][_name]['ros__parameters']['bridge'] = port_maps

    # print(config_dict)

    # create temp file for config load
    result_config_param = _create_params_file_from_dict(config_dict)
    # print(type(result_config_param))
    # print(result_config_param)

    return result_config_param


def connect_to_simulator(_target_model_name):

    delay = 3.0

    while True:

        try:
            ws = create_connection("ws://" + SIM_BIRDGE_IP + ":" + str(SIM_BRIDGE_SERVICE_PORT) + "/control")
            message = "{'command':'device_list', 'filter':'" + _target_model_name + "'}"
            ws.send(message)
            # print("send '%s'" % message)
            result = ws.recv()
            print("Received '%s'" % result)
            ws.close()

            return result;

        except ConnectionRefusedError as err:
            print("=> Failed to connect to CLOiSim: {}\n".format(err))
            print("Try to reconnect to CLOiSim after {} sec: {}".format(delay, err))
            time.sleep(delay)


def get_target_device_list(_target_model_name):

    delay = 2.0

    while True:

        result = connect_to_simulator(_target_model_name)

        device_list = json.loads(result)

        try:
            target_device_list = device_list["result"][_target_model_name]
            # print(target_device_list)
            return target_device_list

        except Exception as inst:
            # print(type(inst))
            # print(inst)
            print("Target robot name is invalid: " + _target_model_name + ", it may be not loaded yet.")
            print("Retry to get device list after {} sec".format(delay))
            time.sleep(delay)
