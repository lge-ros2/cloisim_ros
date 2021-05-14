/**
 *  @file   gps.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 GPS class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_gps/gps.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace cloisim_ros;

Gps::Gps(const rclcpp::NodeOptions &options_, const string node_name_, const string namespace_)
    : Base(node_name_, namespace_, options_, 2)
{
  topic_name_ = "navsatfix";

  Start();
}

Gps::Gps(const string namespace_)
    : Gps(rclcpp::NodeOptions(), "cloisim_ros_gps", namespace_)
{
}

Gps::~Gps()
{
  Stop();
}

void Gps::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  hashKeySub_ = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", hashKeySub_.c_str());

  // Get frame for message
  const auto frame_id = GetFrameId();
  msg_navsat.header.frame_id = frame_id;

  // Fill covariances
  // TODO: need to applying noise
  msg_navsat.position_covariance[0] = 0.0001f;
  msg_navsat.position_covariance[4] = 0.0001f;
  msg_navsat.position_covariance[8] = 0.0001f;
  msg_navsat.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg_navsat.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg_navsat.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  auto pBridgeData = GetBridge(0);
  auto pBridgeInfo = GetBridge(1);

  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeySub_ + "Data");
  }

  if (pBridgeInfo != nullptr)
  {
    pBridgeInfo->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeySub_ + "Info");

    GetRos2Parameter(pBridgeInfo);

    const auto transform = GetObjectTransform(pBridgeInfo);
    SetupStaticTf2(transform, frame_id + "_link");
  }

  // ROS2 Publisher
  pubNav = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, rclcpp::SensorDataQoS());
}

void Gps::Deinitialize()
{
}

void Gps::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  if (!pbBuf_.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBuf_.time().sec(), pbBuf_.time().nsec());

  // Fill message with latest sensor data
  msg_navsat.header.stamp = m_simTime;
  msg_navsat.latitude = pbBuf_.latitude_deg();
  msg_navsat.longitude = pbBuf_.longitude_deg();
  msg_navsat.altitude = pbBuf_.altitude();

  pubNav->publish(msg_navsat);
}