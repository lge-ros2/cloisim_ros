/**
 *  @file   main.cpp
 *  @date   2020-05-13
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Camera Driver class for simulator
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#include "depth_camera_driver_sim/DepthCameraDriverSim.hpp"

using namespace std::literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(100ms);

  auto node = std::make_shared<DepthCameraDriverSim>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
