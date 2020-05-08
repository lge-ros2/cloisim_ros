#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import lifecycle_msgs.msg
import launch.actions
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription


def generate_launch_description():

    # Get the launch directory
    _pkg_name = "simdevice_bringup"
    config_dir = os.path.join(get_package_share_directory(_pkg_name), 'config')

    config_params = os.path.join(config_dir, 'params.elevator_system.yaml')

    _node_executable = 'elevator_system'
    node = LifecycleNode(
        package=_node_executable,
        node_name=_node_executable,
        node_executable=_node_executable,
        parameters=[config_params],
        output='screen')

    # When the node reaches the 'inactive' state, make it to the 'activate'
    evt_hwnd = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[
                LogInfo(
                    msg=_node_executable + " reached the 'inactive' state, 'activating'."),
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

    stdout_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_envvar)

    ld.add_action(evt_hwnd)
    ld.add_action(node)
    ld.add_action(emit_configure_transition)

    return ld
