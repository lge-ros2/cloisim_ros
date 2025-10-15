/**
 *  @file   contact.cpp
 *  @date   2025-01-23
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 contact class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2025 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_contact/contact.hpp"
#include <geometry_msgs/msg/wrench.hpp>


using string = std::string;

namespace cloisim_ros
{
Contact::Contact(
  const rclcpp::NodeOptions & options_, const string node_name,
  const string namespace_)
: Base(node_name, namespace_, options_)
{
  topic_name_ = "contacts";

  Start();
}

Contact::Contact(const string namespace_)
: Contact(rclcpp::NodeOptions(), "cloisim_ros_contact", namespace_)
{
}

Contact::~Contact() {Stop();}

void Contact::Initialize()
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
    const auto frame_id = GetFrameId("contact_link");
    msg_contacts_.header.frame_id = frame_id;

    auto parent_frame_id = std::string("base_link");
    auto transform_pose = GetObjectTransform(info_bridge_ptr, parent_frame_id);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose, parent_frame_id);
  }

  // ROS2 Publisher
  const auto new_topic = GetPartsName() + "/" + topic_name_;
  pub_ =
    this->create_publisher<ros_gz_interfaces::msg::Contacts>(new_topic, rclcpp::SensorDataQoS());

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(data_bridge_ptr,
        bind(&Contact::PublishData, this, std::placeholders::_1));
  }
}

void Contact::PublishData(const string & buffer)
{
  if (!pb_buf_.ParseFromString(buffer)) {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  // SetTime(pb_buf_.time());
  msg::Convert(pb_buf_, msg_contacts_);

  pub_->publish(msg_contacts_);
}

}  // namespace cloisim_ros
