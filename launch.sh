#!/bin/bash
domain_id=7
docker run -it --rm --net=host -e ROS_DOMAIN_ID=${domain_id} cloisim_ros $@
