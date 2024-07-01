"""
LGE Advanced Robotics Laboratory
Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
All Rights are Reserved.

SPDX-License-Identifier: MIT
"""

import launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    _single_mode = LaunchConfiguration('single_mode')
    _target_model = LaunchConfiguration('target_model')
    _target_parts_type = LaunchConfiguration('target_parts_type')
    _target_parts_name = LaunchConfiguration('target_parts_name')
    _enable_tf_micom = LaunchConfiguration('micom.enable_tf')
    _scan = LaunchConfiguration('scan')
    _cmd_vel = LaunchConfiguration('cmd_vel')
    _odom = LaunchConfiguration('odom')
    _imu = LaunchConfiguration('imu')
    _navsatfix = LaunchConfiguration('navsatfix')

    declare_launch_argument_sm = DeclareLaunchArgument(
        'single_mode',
        default_value='False',
        description='whether to use single mode')

    declare_launch_argument_tm = DeclareLaunchArgument(
        'target_model',
        default_value='',
        description='specify the target model you want')

    declare_launch_argument_tpt = DeclareLaunchArgument(
        'target_parts_type',
        default_value='',
        description='specify the type of target parts you want')

    declare_launch_argument_tpn = DeclareLaunchArgument(
        'target_parts_name',
        default_value='',
        description='specify the name of target parts you want')

    declare_launch_argument_etm = DeclareLaunchArgument(
        'micom.enable_tf',
        default_value='True',
        description='whether to use tf/tf_static for Micom')

    declare_launch_argument_sc = DeclareLaunchArgument(
        'scan',
        default_value='scan',
        description='specify scan topic you want')

    declare_launch_argument_cmdvel = DeclareLaunchArgument(
        'cmd_vel',
        default_value='cmd_vel',
        description='specify cmd_vel topic you want')

    declare_launch_argument_odom = DeclareLaunchArgument(
        'odom',
        default_value='odom',
        description='specify odom topic you want')

    declare_launch_argument_imu = DeclareLaunchArgument(
        'imu',
        default_value='imu',
        description='specify imu/data topic you want')

    declare_launch_argument_navsatfix = DeclareLaunchArgument(
        'navsatfix',
        default_value='navsatfix',
        description='specify navsatfix topic you want')

    stdout_log_use_stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')

    stdout_log_buf_stream_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    cloisim_ros_cmd = Node(
        package="cloisim_ros_bringup",
        executable="bringup",
        output='screen',
        remappings=[('scan', _scan),
                    ('cmd_vel', _cmd_vel),
                    ('odom', _odom),
                    ('imu/data_raw', _imu),
                    ('navsatfix', _navsatfix)],
        parameters=[{'single_mode': _single_mode,
                     'target_model': ParameterValue(_target_model, value_type=str),
                     'target_parts_type': ParameterValue(_target_parts_type, value_type=str),
                     'target_parts_name': ParameterValue(_target_parts_name, value_type=str),
                     'micom.enable_tf': _enable_tf_micom}])

    # Create the launch description and populate
    ld = launch.LaunchDescription()

     # Set environment variables
    ld.add_action(stdout_log_use_stdout_envvar)
    ld.add_action(stdout_log_buf_stream_envvar)
    ld.add_action(declare_launch_argument_sm)
    ld.add_action(declare_launch_argument_tm)
    ld.add_action(declare_launch_argument_tpt)
    ld.add_action(declare_launch_argument_tpn)
    ld.add_action(declare_launch_argument_sc)
    ld.add_action(declare_launch_argument_cmdvel)
    ld.add_action(declare_launch_argument_odom)
    ld.add_action(declare_launch_argument_imu)
    ld.add_action(declare_launch_argument_navsatfix)
    ld.add_action(declare_launch_argument_etm)
    ld.add_action(cloisim_ros_cmd)

    return ld
