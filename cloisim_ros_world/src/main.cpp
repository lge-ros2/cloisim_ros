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

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cloisim_ros::World>();
  rclcpp::sleep_for(100ms);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}