/**
 *  @file   main_cam.cpp
 *  @date   2024-02-05
 *  @author Hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Camera class for cloisim
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_camera/camera.hpp"
#include <cloisim_ros_bringup_param/standalone.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  run_standalone_single_executor<cloisim_ros::Camera>("cloisim_ros_camera", "CAMERA");
  rclcpp::shutdown();
  return 0;
}
