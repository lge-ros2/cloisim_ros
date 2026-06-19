/**
 *  @file   main.cpp
 *  @date   2021-05-07
 *  @author Sungkyu Kang
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 realsense class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright 2020 LG Electronics Inc. , LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_realsense/realsense.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

int main(int argc, char ** argv)
{
  return cloisim_ros::RunNode<cloisim_ros::RealSense>(
    argc, argv, "cloisim_ros_realsense", "REALSENSE");
}
