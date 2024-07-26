/**
 *  @file   main_segcam.cpp
 *  @date   2024-03-01
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Segmentation Camera class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_camera/segmentation_camera.hpp"
#include "cloisim_ros_camera/standalone.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  run_standalone_single_executor<cloisim_ros::SegmentationCamera>(
    "cloisim_ros_segmentationcamera", "SEGMENTCAMERA");
  rclcpp::shutdown();
  return 0;
}
