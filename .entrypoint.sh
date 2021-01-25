#!/bin/bash
set -e
source "/opt/lge-ros2/install/setup.bash"
echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID
exec ros2 run cloisim_ros_bringup cloisim_ros_bringup "$@"