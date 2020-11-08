/**
 *  @file   GPSDriverSim.cpp
 *  @date   2020-06-26
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 GPS Driver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "gps_driver_sim/GPSDriverSim.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

GPSDriverSim::GPSDriverSim(const string node_name)
    : DriverSim(node_name, 2)
{
  topic_name_ = "navsatfix";
  frame_id_ = "gps";

  Start();
}

GPSDriverSim::~GPSDriverSim()
{
  Stop();
}

void GPSDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Data", portData_, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  m_hashKeySub = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  // Get frame for message
  msg_navsat.header.frame_id = frame_id_;

  // Fill covariances
  // TODO: need to applying noise
  msg_navsat.position_covariance[0] = 0.0001f;
  msg_navsat.position_covariance[4] = 0.0001f;
  msg_navsat.position_covariance[8] = 0.0001f;
  msg_navsat.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg_navsat.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg_navsat.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  auto pSimBridgeData = GetSimBridge(0);
  auto pSimBridgeInfo = GetSimBridge(1);

  if (pSimBridgeData != nullptr)
  {
    pSimBridgeData->Connect(SimBridge::Mode::SUB, portData_, m_hashKeySub + "Data");
  }

  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, portInfo, m_hashKeySub + "Info");

    GetRos2Parameter(pSimBridgeInfo);

    const auto transform = GetObjectTransform(pSimBridgeInfo);
    SetupStaticTf2(transform, frame_id_ + "_link");
  }

  // ROS2 Publisher
  pubNav = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, rclcpp::SensorDataQoS());
}

void GPSDriverSim::Deinitialize()
{
  DisconnectSimBridges();
}

void GPSDriverSim::UpdateData(const uint bridge_index)
{
  auto simBridge = GetSimBridge(bridge_index);
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = simBridge->Receive(&pBuffer, bufferLength, false);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));
    return;
  }

  if (!m_pbBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(m_pbBuf.time().sec(), m_pbBuf.time().nsec());

  // Fill message with latest sensor data
  msg_navsat.header.stamp = m_simTime;
  msg_navsat.latitude = m_pbBuf.latitude_deg();
  msg_navsat.longitude = m_pbBuf.longitude_deg();
  msg_navsat.altitude = m_pbBuf.altitude();

  pubNav->publish(msg_navsat);
}