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

GPSDriverSim::GPSDriverSim()
    : DriverSim("gps_driver_sim", 2)
{
  Start();
}

GPSDriverSim::~GPSDriverSim()
{
  Stop();
}

void GPSDriverSim::Initialize()
{
  string topic_name_;
  string frame_id_;

  get_parameter_or("topic_name", topic_name_, string("scan"));
  get_parameter_or("frame_id", frame_id_, string("gps"));

  DBG_SIM_INFO("[CONFIG] topic_name: %s", topic_name_.c_str());
  DBG_SIM_INFO("[CONFIG] frame_id: %s", frame_id_.c_str());

  m_hashKeySub = GetRobotName() + GetPartsName();
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

  // ROS2 Publisher
  pubNav = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, rclcpp::SensorDataQoS());

  auto pSimBridgeData = GetSimBridge(0);
  auto pSimBridgeInfo = GetSimBridge(1);

  if (pSimBridgeData != nullptr)
  {
    pSimBridgeData->Connect(SimBridge::Mode::SUB, m_hashKeySub);
  }

  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, m_hashKeySub + "Info");
    const auto transform = GetObjectTransform(pSimBridgeInfo);
    SetupStaticTf2Message(transform, frame_id_);
  }
}

void GPSDriverSim::Deinitialize()
{
  DisconnectSimBridges();
}

void GPSDriverSim::SetupStaticTf2Message(const gazebo::msgs::Pose transform, const string frame_id)
{
  geometry_msgs::msg::TransformStamped gps_tf;
  gps_tf.header.frame_id = "base_link";
  gps_tf.child_frame_id = frame_id;
  gps_tf.transform.translation.x = transform.position().x();
  gps_tf.transform.translation.y = transform.position().y();
  gps_tf.transform.translation.z = transform.position().z();
  gps_tf.transform.rotation.x = transform.orientation().x();
  gps_tf.transform.rotation.y = transform.orientation().y();
  gps_tf.transform.rotation.z = transform.orientation().z();
  gps_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(gps_tf);
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

    // try reconnect1ion
    if (IsRunThread())
    {
      simBridge->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
    }

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