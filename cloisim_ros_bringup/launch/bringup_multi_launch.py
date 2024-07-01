"""
LGE Advanced Robotics Laboratory
Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
All Rights are Reserved.

SPDX-License-Identifier: MIT
"""

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    bringup_dir = get_package_share_directory('cloisim_ros_bringup')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, "launch", "bringup_launch.py"])
        ),
        launch_arguments={'single_mode': False}.items()
    )

    ld = launch.LaunchDescription()
    ld.add_action(bringup_cmd)

    return ld
