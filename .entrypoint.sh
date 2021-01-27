#!/bin/bash
set -e
source "/opt/lge-ros2/install/setup.bash"

echo "Check ROS_DOMAIN_ID here!!"
echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID

exec ros2 launch cloisim_ros_bringup bringup.launch.py "$@"