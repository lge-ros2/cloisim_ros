/**
 *  @file   main.cpp
 *  @date   2026-04-24
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls logical camera sensor for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_logical_camera/logical_camera.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

int main(int argc, char ** argv)
{
  return cloisim_ros::RunNode<cloisim_ros::LogicalCamera>(
    argc, argv, "cloisim_ros_logical_camera", "LOGICALCAMERA");
}
