/**
 *  @file   range.cpp
 *  @date   2025-02-05
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Sonar class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_range/range.hpp"

using string = std::string;

namespace cloisim_ros
{

Range::Range(const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: Base(node_name, namespace_, options_)
  , radiation_type_(sensor_msgs::msg::Range::ULTRASOUND)
{
  topic_name_ = "range";

  Start();
}

Range::Range(const string namespace_)
: Range(rclcpp::NodeOptions(), "cloisim_ros_range", namespace_)
{
}

Range::~Range() {Stop();}

void Range::Initialize()
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
    const auto frame_id = GetFrameId("range_link");
    msg_range_.header.frame_id = frame_id;

    auto parent_frame_id = std::string("base_link");
    auto transform_pose = GetObjectTransform(info_bridge_ptr, parent_frame_id);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose, parent_frame_id);
  }

  // ROS2 Publisher
  const auto new_topic = GetPartsName() + "/" + topic_name_;
  pub_ = this->create_publisher<sensor_msgs::msg::Range>(new_topic, rclcpp::SensorDataQoS());

  const auto new_pose_topic = GetPartsName() + "/" + topic_name_ + "/pose";
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    new_pose_topic,
    rclcpp::SensorDataQoS());

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(data_bridge_ptr, bind(&Range::PublishData, this, std::placeholders::_1));
  }
}

void Range::PublishData(const string & buffer)
{
  if (!pb_buf_.ParseFromString(buffer)) {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.time());

  // Fill message with latest sensor data
  msg_range_.header.stamp = GetTime();
  msg_range_.header.frame_id = pb_buf_.sonar().frame();

  msg_pose_.header.stamp = GetTime();
  msg_pose_.header.frame_id = pb_buf_.sonar().frame();

  msg::Convert(pb_buf_.sonar().world_pose(), msg_pose_.pose);

  msg_range_.radiation_type = radiation_type_;
  if (pb_buf_.sonar().has_radius()) {msg_range_.field_of_view = pb_buf_.sonar().radius();}
  if (pb_buf_.sonar().has_range_min()) {msg_range_.min_range = pb_buf_.sonar().range_min();}
  if (pb_buf_.sonar().has_range_max()) {msg_range_.max_range = pb_buf_.sonar().range_max();}
  if (pb_buf_.sonar().has_range()) {msg_range_.range = static_cast<float>(pb_buf_.sonar().range());}

  pub_->publish(msg_range_);
  pub_pose_->publish(msg_pose_);
}

}  // namespace cloisim_ros
