/**
 *  @file   main_sonar.cpp
 *  @date   2025-02-05
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls sonar sensor for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_range/range.hpp"
#include <cloisim_ros_bringup_param/standalone.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = run_standalone_single_executor<cloisim_ros::Range>(
    "cloisim_ros_sonar", "SONAR");
  node->SetRadiationType(sensor_msgs::msg::Range::ULTRASOUND);
  rclcpp::shutdown();
  return 0;
}
