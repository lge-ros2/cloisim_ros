/**
 *  @file   Lidar.cpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Lidar class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_lidar/lidar.hpp"
#include <cloisim_msgs/param.pb.h>

using namespace std;
using namespace cloisim_ros;

Lidar::Lidar(const rclcpp::NodeOptions &options_, const string node_name_, const string namespace_)
    : Base(node_name_, namespace_, options_, 2)
{
  topic_name_ = "scan";
  frame_id_ = "base_scan";

  Start();
}

Lidar::Lidar(const string namespace_)
    : Lidar(rclcpp::NodeOptions(), "cloisim_ros_lidar", namespace_)
{
}

Lidar::~Lidar()
{
  Stop();
}

void Lidar::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  hashKeySub_ = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", hashKeySub_.c_str());

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
    SetupStaticTf2(transform, frame_id_);
  }

  // ROS2 Publisher
  pubLaser = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rclcpp::SensorDataQoS());
}

void Lidar::Deinitialize()
{
  DisconnectBridges();
}

void Lidar::UpdateData(const uint bridge_index)
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

  UpdateLaserData();

  pubLaser->publish(msg_Laser);
}

void Lidar::UpdateLaserData()
{
  msg_Laser.header.stamp = m_simTime;
  msg_Laser.header.frame_id = frame_id_;

  msg_Laser.angle_min = pbBuf_.scan().angle_min();
  msg_Laser.angle_max = pbBuf_.scan().angle_max();
  msg_Laser.angle_increment = pbBuf_.scan().angle_step();
  msg_Laser.scan_time = 0; //getScanPeriod();
  msg_Laser.time_increment = 0; //getTimeIncrement();
  msg_Laser.range_min = pbBuf_.scan().range_min();
  msg_Laser.range_max = pbBuf_.scan().range_max();

  const uint32_t num_beams = pbBuf_.scan().count();
  //DBG_SIM_INFO("num_beams:%d", num_beams);

  if (msg_Laser.ranges.size() != num_beams)
    msg_Laser.ranges.resize(num_beams);

  if (msg_Laser.intensities.size() != num_beams)
    msg_Laser.intensities.resize(num_beams);

  for (uint32_t beam_idx = 0; beam_idx < num_beams; beam_idx++)
  {
    //printf("beam_idx:%d %f\n", beam_idx, pbBuf_.scan().ranges(beam_idx));
    msg_Laser.ranges[beam_idx] = pbBuf_.scan().ranges(beam_idx);
    msg_Laser.intensities[beam_idx] = pbBuf_.scan().intensities(beam_idx);
  }
}