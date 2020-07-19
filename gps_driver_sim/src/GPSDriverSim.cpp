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
    : DriverSim("gps_driver_sim")
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
  vector<double> transform_;

  get_parameter_or("topic_name", topic_name_, string("scan"));
  get_parameter_or("frame_id", frame_id_, string("gps"));
  get_parameter_or("transform", transform_, vector<double>({0, 0, 0, 0, 0, 0}));

  DBG_SIM_INFO("[CONFIG] topic_name: %s", topic_name_.c_str());
  DBG_SIM_INFO("[CONFIG] frame_id: %s", frame_id_.c_str());

  m_hashKeySub = GetRobotName() + GetPartsName();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  geometry_msgs::msg::TransformStamped gps_tf;
  tf2::Quaternion convertQuternion;
  convertQuternion.setRPY(transform_[3], transform_[4], transform_[5]);
  convertQuternion = convertQuternion.normalize();

  gps_tf.header.frame_id = "base_link";
  gps_tf.child_frame_id = frame_id_;
  gps_tf.transform.translation.x = transform_[0];
  gps_tf.transform.translation.y = transform_[1];
  gps_tf.transform.translation.z = transform_[2];
  gps_tf.transform.rotation.x = convertQuternion.x();
  gps_tf.transform.rotation.y = convertQuternion.y();
  gps_tf.transform.rotation.z = convertQuternion.z();
  gps_tf.transform.rotation.w = convertQuternion.w();

  AddStaticTf2(gps_tf);

  // Get frame for message
  msg_navsat.header.frame_id = frame_id_;

  // Fill covariances
  // TODO: need to applying noise
  msg_navsat.position_covariance[0] = 1.0f;
  msg_navsat.position_covariance[4] = 1.0f;
  msg_navsat.position_covariance[8] = 1.0f;
  msg_navsat.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg_navsat.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg_navsat.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // ROS2 Publisher
  pubNav = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, rclcpp::SensorDataQoS());

  GetSimBridge()->Connect(SimBridge::Mode::SUB, m_hashKeySub);
}

void GPSDriverSim::Deinitialize()
{
  GetSimBridge()->Disconnect();
}

void GPSDriverSim::UpdateData()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (IsRunThread())
  {
    const bool succeeded = GetSimBridge()->Receive(&pBuffer, bufferLength, false);

    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

      // try reconnect1ion
      if (IsRunThread())
      {
        GetSimBridge()->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
      }

      continue;
    }

    if (!m_pbBuf.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    m_simTime = rclcpp::Time(m_pbBuf.time().sec(), m_pbBuf.time().nsec());

    // Fill message with latest sensor data
    msg_navsat.header.stamp = m_simTime;
    msg_navsat.latitude = m_pbBuf.latitude_deg();
    msg_navsat.longitude = m_pbBuf.longitude_deg();
    msg_navsat.altitude = m_pbBuf.altitude();

    pubNav->publish(msg_navsat);
  }
}