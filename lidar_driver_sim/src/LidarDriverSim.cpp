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
#include <protobuf/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace gazebo;

LidarDriverSim::LidarDriverSim()
    : DriverSim("lidar_driver_sim", 2)
{
  topic_name_ = "scan";
  frame_id_ = "base_scan";

  Start();
}

LidarDriverSim::~LidarDriverSim()
{
  Stop();
}

void LidarDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Data", portData_, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  m_hashKeySub = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

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
    SetupStaticTf2(transform, frame_id_);
  }

  // ROS2 Publisher
  pubLaser = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rcl_publisher_get_default_options().qos);
}

void LidarDriverSim::Deinitialize()
{
  DisconnectSimBridges();
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
      simBridge->Reconnect(SimBridge::Mode::SUB, portData_, m_hashKeySub);
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