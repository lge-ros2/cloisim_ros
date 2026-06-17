/**
 *  @file   main.cpp
 *  @date   2020-04-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 packages that helps to control unity simulation
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_world/world.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

int main(int argc, char ** argv)
{
  return cloisim_ros::RunNodeSingleMode<cloisim_ros::World>(
    argc, argv, "cloisim_ros_world", "WORLD");
}
