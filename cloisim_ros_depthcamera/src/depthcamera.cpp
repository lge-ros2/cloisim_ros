/**
 *  @file   depthcamera.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Depth Camera class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include "cloisim_ros_depthcamera/depthcamera.hpp"

using namespace std;
using namespace cloisim_ros;

DepthCamera::DepthCamera(const string node_name)
    : Camera(node_name)
{
}

DepthCamera::~DepthCamera()
{
  // DBG_SIM_INFO("Delete");
}

void DepthCamera::Initialize()
{
  Camera::Initialize();

  DBG_SIM_INFO("DepthCamera Initlaization");
  // pubPointCloud = create_publisher<sensor_msgs::msg::PointCloud2>(topic_base_name_ + "/points", rclcpp::QoS(rclcpp::KeepLast(1)));
}

void DepthCamera::Deinitialize()
{
  Camera::Deinitialize();

  DBG_SIM_INFO("DepthCamera Deinitialization");
}

void DepthCamera::UpdateData(const uint bridge_index)
{
  (void)bridge_index;
  Camera::UpdateData();
}