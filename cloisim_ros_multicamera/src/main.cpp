/**
 *  @file   main.cpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 multi camera class for simulator
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_multicamera/multicamera.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto cam_node = std::make_shared<cloisim_ros::MultiCamera>();
  rclcpp::spin(cam_node);
  rclcpp::shutdown();
  return 0;
}
