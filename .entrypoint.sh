#!/bin/bash
set -e
source "/opt/lge-ros2/install/setup.bash"
exec ros2 launch cloisim_ros_bringup "$@"
# exec ros2 run cloisim_ros_bringup cloisim_ros_bringup "$@"