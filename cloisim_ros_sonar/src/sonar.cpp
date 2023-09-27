/**
 *  @file   sonar.cpp
 *  @date   2023-05-21
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

#include "cloisim_ros_sonar/sonar.hpp"
#include <cloisim_ros_base/helper.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

namespace cloisim_ros
{

Sonar::Sonar(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
{
  topic_name_ = "range";

  Start();
}

Sonar::Sonar(const string namespace_)
    : Sonar(rclcpp::NodeOptions(), "cloisim_ros_sonar", namespace_)
{
}

Sonar::~Sonar()
{
  Stop();
}

void Sonar::Initialize()
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
    const auto frame_id = GetFrameId("sonar_link");
    msg_range_.header.frame_id = frame_id;

    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);
  }

  // ROS2 Publisher
  const auto new_topic = GetPartsName() + "/" + topic_name_;
  pub_ = this->create_publisher<sensor_msgs::msg::Range>(new_topic, rclcpp::SensorDataQoS());

  if (data_bridge_ptr != nullptr)
  {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&Sonar::PublishData, this, std::placeholders::_1));
  }
}

void Sonar::PublishData(const string &buffer)
{
  if (!pb_buf_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.time());

  // Fill message with latest sensor data
  msg_range_.header.stamp = GetTime();
  msg_range_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  if (pb_buf_.sonar().has_radius())
    msg_range_.field_of_view = pb_buf_.sonar().radius();
  if (pb_buf_.sonar().has_range_min())
    msg_range_.min_range = pb_buf_.sonar().range_min();
  if (pb_buf_.sonar().has_range_max())
    msg_range_.max_range = pb_buf_.sonar().range_max();
  if (pb_buf_.sonar().has_range())
    msg_range_.range = static_cast<float>(pb_buf_.sonar().range());

  pub_->publish(msg_range_);
}

}  // namespace cloisim_ros
