/**
 *  @file   imu.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 imu class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_imu/imu.hpp"

using string = std::string;

namespace cloisim_ros
{

Imu::Imu(const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: Base(node_name, namespace_, options_)
{
  topic_name_ = "imu/data_raw";

  Start();
}

Imu::Imu(const string namespace_)
: Imu(rclcpp::NodeOptions(), "cloisim_ros_imu", namespace_) {}

Imu::~Imu() {Stop();}

void Imu::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  DBG_SIM_INFO("hashKey: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    // Get frame for message
    const auto frame_id = GetFrameId("imu_link");
    msg_imu_.header.frame_id = frame_id;

    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);
  }

  std::fill(begin(msg_imu_.orientation_covariance), end(msg_imu_.orientation_covariance), 0.0);
  std::fill(
    begin(msg_imu_.angular_velocity_covariance), end(msg_imu_.angular_velocity_covariance), 0.0);
  std::fill(
    begin(msg_imu_.linear_acceleration_covariance), end(msg_imu_.linear_acceleration_covariance),
    0.0);

  // ROS2 Publisher
  pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name_, rclcpp::SensorDataQoS());

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&Imu::PublishData, this, std::placeholders::_1));
  }
}

void Imu::PublishData(const string & buffer)
{
  if (!pb_buf_.ParseFromString(buffer)) {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.stamp());

  // Fill message with latest sensor data
  msg_imu_.header.stamp = GetTime();

  msg::Convert(pb_buf_.orientation(), msg_imu_.orientation);
  msg::Convert(pb_buf_.angular_velocity(), msg_imu_.angular_velocity);
  msg::Convert(pb_buf_.linear_acceleration(), msg_imu_.linear_acceleration);

  pub_->publish(msg_imu_);
}

}  // namespace cloisim_ros
