/**
 *  @file   main.cpp
 *  @date   2020-05-13
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 realsense Driver class for simulator
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#include "realsense_driver_sim/RealSenseDriverSim.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealSenseDriverSim>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
