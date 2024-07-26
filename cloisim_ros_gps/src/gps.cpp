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

using string = std::string;

namespace cloisim_ros
{

Gps::Gps(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
    , topic_name_heading_("navsatfix/heading")
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
  DBG_SIM_INFO("hashKey: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    // Get frame for message
    const auto frame_id = GetFrameId("gps_link");
    msg_gps_.header.frame_id = frame_id;

    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);
  }

  // Fill covariances
  // TODO(@hyunseok-yang): need to applying noise
  msg_gps_.position_covariance[0] = 0.0001f;
  msg_gps_.position_covariance[4] = 0.0001f;
  msg_gps_.position_covariance[8] = 0.0001f;
  msg_gps_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg_gps_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg_gps_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  std::fill(begin(msg_heading_.orientation_covariance),
            end(msg_heading_.orientation_covariance), 0.0);
  std::fill(begin(msg_heading_.angular_velocity_covariance),
            end(msg_heading_.angular_velocity_covariance), 0.0);
  std::fill(begin(msg_heading_.linear_acceleration_covariance),
            end(msg_heading_.linear_acceleration_covariance), 0.0);

  msg_heading_.orientation_covariance[0] = 0.0001f;
  msg_heading_.orientation_covariance[4] = 0.0001f;
  msg_heading_.orientation_covariance[8] = 0.0001f;

  // ROS2 Publisher
  pub_gps_ =
      this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name_, rclcpp::SensorDataQoS());

  pub_heading_ =
      this->create_publisher<sensor_msgs::msg::Imu>(topic_name_heading_, rclcpp::SensorDataQoS());

  if (data_bridge_ptr != nullptr)
  {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&Gps::PublishData, this, std::placeholders::_1));
  }
}

void Gps::PublishData(const string &buffer)
{
  if (!pb_buf_gps_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_gps_.time());

  // Fill message with latest sensor data
  msg_gps_.header.stamp = GetTime();
  msg_gps_.latitude = pb_buf_gps_.latitude_deg();
  msg_gps_.longitude = pb_buf_gps_.longitude_deg();
  msg_gps_.altitude = pb_buf_gps_.altitude();

  pub_gps_->publish(msg_gps_);

  msg_heading_.header.stamp = GetTime();

  if (pb_buf_gps_.has_heading())
  {
    ConvertCLOiSimToRos2(
        pb_buf_gps_.heading().orientation(),
        msg_heading_.orientation);
    ConvertCLOiSimToRos2(
        pb_buf_gps_.heading().angular_velocity(),
        msg_heading_.angular_velocity);
    ConvertCLOiSimToRos2(
        pb_buf_gps_.heading().linear_acceleration(),
        msg_heading_.linear_acceleration);
  }

  pub_heading_->publish(msg_heading_);
}

}  // namespace cloisim_ros
