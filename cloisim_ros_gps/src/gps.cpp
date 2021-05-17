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
    : Base(node_name_, namespace_, options_)
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

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  DBG_SIM_INFO("hash Key: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto pBridgeData = CreateBridge(hashKeyData);
  auto pBridgeInfo = CreateBridge(hashKeyInfo);

  if (pBridgeInfo != nullptr)
  {
    pBridgeInfo->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(pBridgeInfo);

    // Get frame for message
    const auto frame_id = GetFrameId();
    msgNavSat.header.frame_id = frame_id;

    const auto transform = GetObjectTransform(pBridgeInfo);
    SetupStaticTf2(transform, frame_id + "_link");
  }

  // Fill covariances
  // TODO: need to applying noise
  msgNavSat.position_covariance[0] = 0.0001f;
  msgNavSat.position_covariance[4] = 0.0001f;
  msgNavSat.position_covariance[8] = 0.0001f;
  msgNavSat.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msgNavSat.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msgNavSat.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // ROS2 Publisher
  pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, rclcpp::SensorDataQoS());

  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    CreatePublisherThread(pBridgeData);
  }
}

void Gps::Deinitialize()
{
}

void Gps::UpdatePublishingData(const string &buffer)
{
  if (!pbGps.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetSimTime(pbGps.time());

  // Fill message with latest sensor data
  msgNavSat.header.stamp = GetSimTime();
  msgNavSat.latitude = pbGps.latitude_deg();
  msgNavSat.longitude = pbGps.longitude_deg();
  msgNavSat.altitude = pbGps.altitude();

  pub->publish(msgNavSat);
}