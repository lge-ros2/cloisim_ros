 /**
 *  @file   main.cpp
 *  @date   2020-05-27
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
#include "micom_driver_sim/MicomDriverSim.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MicomDriverSim>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}