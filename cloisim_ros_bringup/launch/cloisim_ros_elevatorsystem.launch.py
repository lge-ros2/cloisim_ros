#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import lifecycle_msgs.msg
import launch_ros
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _name = LaunchConfiguration('name')
    _parameters = LaunchConfiguration('parameters')

    _package_name = 'cloisim_ros_elevatorsystem'

    node = LifecycleNode(
        package=_package_name,
        executable=_package_name,
        name=_name,
        parameters=[_parameters],
        output='screen')

    # When the node reaches the 'inactive' state, make it to the 'activate'
    evt_hwnd = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[
                LogInfo(
                    msg=_package_name + " reached the 'inactive' state, 'activating'."),
                EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE)
                )
            ]
        )
    )

    # Make the node take the 'configure' transition.
    emit_configure_transition = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    declare_launch_argument_nn = DeclareLaunchArgument(
        'name',
        default_value='camera',
        description='it is node name')

    stdout_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_envvar)

    ld.add_action(evt_hwnd)

    ld.add_action(declare_launch_argument_nn)

    ld.add_action(node)

    ld.add_action(emit_configure_transition)

    return ld
