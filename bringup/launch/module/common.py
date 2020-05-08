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


def _create_params_file_from_dict(params):
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
        param_file_path = h.name
        yaml.dump(params, h, default_flow_style=False)
        return param_file_path


def _get_modified_config_dict(config_dict, model_name, node_name):

    try:
        config_dict[node_name]['ros__parameters']['sim']['model'] = model_name
    except:
        print("")
        print("ros__parameter:")
        print("  sim:")
        print("    model:")
        print("it is mandatory in yaml configuration file")
        print("")
        raise

    return config_dict


def _set_remapping_list(namespace, remapping_topic_list):

    result_remapping_list = []

    ns_for_remap = '/' + namespace

    # set topic remap list with prefix
    for remap_topic in remapping_topic_list:
        result_remapping_list.append(
            tuple([remap_topic, ns_for_remap + remap_topic]))

    return result_remapping_list


def get_modified_params_with_ns_and_remapping_list(config_file_path, target_node_name):

    result_config_param = config_file_path

    _namespace = find_robot_name()

    _remapping_list = []

    if type(_namespace) is str and len(_namespace.strip()) > 0:

        with open(config_file_path, 'r') as stream:
            config_dict = yaml.safe_load(stream)

        # get_modify_config_dict
        config_dict = _get_modified_config_dict(
            config_dict, _namespace, target_node_name)

        # get remmapping list
        try:
            _remapping_topic_list = config_dict[target_node_name]['ros__parameters']["remapping_list"]
            _remapping_list = _set_remapping_list(
                _namespace, _remapping_topic_list)
        except:
            print("No remapping list!")

        # capsule config with namespace
        config_dict_with_ns = {}
        config_dict_with_ns[_namespace] = config_dict

        # create temp file for config load
        result_config_param = _create_params_file_from_dict(
            config_dict_with_ns)

    return (result_config_param, _remapping_list)


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
