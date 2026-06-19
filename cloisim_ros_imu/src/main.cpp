/**
 *  @file   main.cpp
 *  @date   2020-06-26
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls imu sensor for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_imu/imu.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

int main(int argc, char ** argv)
{
  return cloisim_ros::RunNode<cloisim_ros::Imu>(
    argc, argv, "cloisim_ros_imu", "IMU");
}
