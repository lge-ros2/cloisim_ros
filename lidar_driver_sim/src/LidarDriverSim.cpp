/**
 *  @file   LidarDriverSim.cpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Lidar Driver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "lidar_driver_sim/LidarDriverSim.hpp"
#include <unistd.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#define MAX_UPPER_ANGLE 3.1415926535
#define MAX_LOWER_ANGLE -MAX_UPPER_ANGLE

using namespace std;
using namespace chrono_literals;

LidarDriverSim::LidarDriverSim()
    : DriverSim("lidar_driver_sim", 2),
    m_bIntensity(false),
    m_fLowerAngle(MAX_LOWER_ANGLE),
    m_fUpperAngle(MAX_UPPER_ANGLE)
{
  Start();
}

LidarDriverSim::~LidarDriverSim()
{
  Stop();
}

void LidarDriverSim::Initialize()
{
  string topic_name_;

  get_parameter_or("topic_name", topic_name_, string("scan"));
  get_parameter_or("frame_id", frame_id_, string("base_scan"));

  get_parameter("intensity", m_bIntensity);
  get_parameter("filter.lower_angle", m_fLowerAngle);
  get_parameter("filter.upper_angle", m_fUpperAngle);

  DBG_SIM_INFO("[CONFIG] topic_name: %s", topic_name_.c_str());
  DBG_SIM_INFO("[CONFIG] frame_id: %s", frame_id_.c_str());
  DBG_SIM_INFO("[CONFIG] intensity: %d, filter.lower_angle: %f, filter.upper_angle: %f",
               m_bIntensity, m_fLowerAngle, m_fUpperAngle);

  m_hashKeySub = GetRobotName() + GetPartsName();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

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

  // ROS2 Publisher
  pubLaser = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rclcpp::SensorDataQoS());
}

void LidarDriverSim::Deinitialize()
{
  DisconnectSimBridges();
}

void LidarDriverSim::SetupStaticTf2Message(const gazebo::msgs::Pose transform, const string frame_id)
{
  geometry_msgs::msg::TransformStamped scan_tf;
  scan_tf.header.frame_id = "base_link";
  scan_tf.child_frame_id = frame_id;
  scan_tf.transform.translation.x = transform.position().x();
  scan_tf.transform.translation.y = transform.position().y();
  scan_tf.transform.translation.z = transform.position().z();
  scan_tf.transform.rotation.x = transform.orientation().x();
  scan_tf.transform.rotation.y = transform.orientation().y();
  scan_tf.transform.rotation.z = transform.orientation().z();
  scan_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(scan_tf);
}

void LidarDriverSim::UpdateData(const uint bridge_index)
{
  auto simBridge = GetSimBridge(bridge_index);
  if (simBridge == nullptr)
  {
    return;
  }

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

  UpdateLaserData();

  pubLaser->publish(msg_Laser);
}

void LidarDriverSim::UpdateLaserData()
{
  msg_Laser.header.stamp = m_simTime;
  msg_Laser.header.frame_id = frame_id_;

  msg_Laser.angle_min = m_pbBuf.scan().angle_min();
  msg_Laser.angle_max = m_pbBuf.scan().angle_max();
  msg_Laser.angle_increment = m_pbBuf.scan().angle_step();
  msg_Laser.scan_time = 0; //getScanPeriod();
  msg_Laser.time_increment = 0; //getTimeIncrement();
  msg_Laser.range_min = m_pbBuf.scan().range_min();
  msg_Laser.range_max = m_pbBuf.scan().range_max();

  const uint32_t num_beams = m_pbBuf.scan().count();
  //DBG_SIM_INFO("num_beams:%d", num_beams);

  if (msg_Laser.ranges.size() != num_beams)
    msg_Laser.ranges.resize(num_beams);

  if (msg_Laser.intensities.size() != num_beams)
    msg_Laser.intensities.resize(num_beams);

  for (uint32_t beam_idx = 0; beam_idx < num_beams; beam_idx++)
  {
    //printf("beam_idx:%d %f\n", beam_idx, m_pbBuf.scan().ranges(beam_idx));
    msg_Laser.ranges[beam_idx] = m_pbBuf.scan().ranges(beam_idx);
    msg_Laser.intensities[beam_idx] = m_pbBuf.scan().intensities(beam_idx);
  }
}