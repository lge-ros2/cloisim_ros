/**
 *  @file   main.cpp
 *  @date   2019-03-28
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls lidar sensor for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#include "lidar_driver_sim/CLidarDriverSim.hpp"
#include <iostream>

using namespace std::literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(1500ms);

  auto oLidar = std::make_shared<CLidarDriverSim>();

  rclcpp::spin(oLidar);
  rclcpp::shutdown();

  return 0;
}
