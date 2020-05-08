/**
 *  @file   main.cpp
 *  @date   2019-03-28
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls Micom board for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#include "micom_driver_sim/CMicomDriverSim.hpp"

using namespace std::literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(1000ms);

  auto micomDriverSimNode = std::make_shared<CMicomDriverSim>();
  rclcpp::spin(micomDriverSimNode);
  rclcpp::shutdown();

  return 0;
}
