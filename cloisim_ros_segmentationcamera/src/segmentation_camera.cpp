/**
 *  @file   segmentation_camera.cpp
 *  @date   2024-03-01
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Segmentation Camera class for simulator
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_segmentationcamera/segmentation_camera.hpp"

using namespace std;

namespace cloisim_ros
{

SegmentationCamera::SegmentationCamera(
    const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Camera(options_, node_name, namespace_)
{
}

SegmentationCamera::SegmentationCamera(
    const string node_name, const string namespace_)
    : SegmentationCamera(rclcpp::NodeOptions(), node_name, namespace_)
{
}

SegmentationCamera::~SegmentationCamera()
{
  // DBG_SIM_INFO("Delete");
}

void SegmentationCamera::Initialize()
{
  Camera::Initialize();

  DBG_SIM_INFO("SegmentationCamera Initlaization");
  // pubPointCloud = create_publisher<sensor_msgs::msg::PointCloud2>(topic_base_name_ + "/points", rclcpp::QoS(rclcpp::KeepLast(1)));
}

void SegmentationCamera::Deinitialize()
{
  Camera::Deinitialize();

  DBG_SIM_INFO("SegmentationCamera Deinitialization");
}

void SegmentationCamera::PublishData(const string &buffer)
{
  Camera::PublishData(buffer);
}

}  // namespace cloisim_ros
