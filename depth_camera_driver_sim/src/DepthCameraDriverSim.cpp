/**
 *  @file   DepthCameraDriverSim.cpp
 *  @date   2020-03-20
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Depth Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include "depth_camera_driver_sim/DepthCameraDriverSim.hpp"

using namespace std;
using namespace chrono_literals;

DepthCameraDriverSim::DepthCameraDriverSim()
    : CameraDriverSim("depth_camera_driver_sim")
{
}

DepthCameraDriverSim::~DepthCameraDriverSim()
{
  DBG_SIM_INFO("Delete");
}

void DepthCameraDriverSim::Initialize()
{
  CameraDriverSim::Initialize();

  DBG_SIM_INFO("DepthCamera Initlaization");
  // pubPointCloud = create_publisher<sensor_msgs::msg::PointCloud2>(topic_base_name_ + "/points", rclcpp::QoS(rclcpp::KeepLast(1)));
}

void DepthCameraDriverSim::Deinitialize()
{
  CameraDriverSim::Deinitialize();

  DBG_SIM_INFO("DepthCamera Deinitialization");
}

void DepthCameraDriverSim::UpdateData()
{
  CameraDriverSim::UpdateData();
}