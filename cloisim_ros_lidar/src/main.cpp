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

#include "cloisim_ros_lidar/lidar.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cloisim_ros::Lidar>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
