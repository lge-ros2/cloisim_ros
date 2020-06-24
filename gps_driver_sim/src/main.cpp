/**
 *  @file   main.cpp
 *  @date   2020-06-26
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls gps sensor for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "gps_driver_sim/GPSDriverSim.hpp"
using namespace std::literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPSDriverSim>();
  rclcpp::spin(node);
  rclcpp::sleep_for(100ms);
  rclcpp::shutdown();
  return 0;
}
