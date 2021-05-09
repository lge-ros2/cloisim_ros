#!/bin/bash
docker run -it --rm --net=host -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} cloisim_ros $@