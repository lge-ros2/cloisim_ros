/**
 *  @file   logical_camera.cpp
 *  @date   2026-04-24
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 logical camera class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_logical_camera/logical_camera.hpp"

#include <cmath>

#include <cloisim_ros_base/param_helper.hpp>

using string = std::string;

namespace cloisim_ros
{

LogicalCamera::LogicalCamera(
  const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: Base(node_name, namespace_, options_)
{
  topic_name_ = "logical_camera";

  Start();
}

LogicalCamera::LogicalCamera(const string namespace_)
: LogicalCamera(rclcpp::NodeOptions(), "cloisim_ros_logical_camera", namespace_) {}

LogicalCamera::~LogicalCamera() {Stop();}

void LogicalCamera::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  LOG_I(this, "hashKey: data(" << hashKeyData << ") info(" << hashKeyInfo << ")");

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    const auto frame_id = GetFrameId("logical_camera_link");
    msg_.header.frame_id = frame_id;

    auto parent_frame_id = std::string("base_link");
    auto transform_pose = GetObjectTransform(info_bridge_ptr, parent_frame_id);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose, parent_frame_id);

    SetupCameraInfo(info_bridge_ptr);
  }

  pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    topic_name_, rclcpp::SensorDataQoS());

  pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    topic_name_ + "/camera_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  if (pub_camera_info_ != nullptr) {
    pub_camera_info_->publish(msg_camera_info_);
  }

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(
      data_bridge_ptr,
      bind(&LogicalCamera::PublishData, this, std::placeholders::_1, std::placeholders::_2));
  }
}

void LogicalCamera::PublishData(const void * buffer, int bufferLength)
{
  if (!pb_buf_.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "##Parsing error, size=" << bufferLength);
    return;
  }

  SetTime(pb_buf_.header().stamp());

  UpdateLogicalCameraData();

  pub_->publish(msg_);
}

void LogicalCamera::UpdateLogicalCameraData()
{
  msg_.header.stamp = GetTime();
  msg_.detections.clear();

  for (auto i = 0; i < pb_buf_.model_size(); i++) {
    const auto & pb_model = pb_buf_.model(i);

    vision_msgs::msg::Detection3D detection;
    detection.header = msg_.header;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = pb_model.name();
    hypothesis.hypothesis.score = 1.0;

    msg::Convert(pb_model.pose(), hypothesis.pose.pose);

    detection.results.push_back(hypothesis);

    msg_.detections.push_back(detection);
  }
}

void LogicalCamera::SetupCameraInfo(zmq::Bridge * const info_bridge_ptr)
{
  cloisim::msgs::Param request_msg;
  cloisim_ros::param::SetName(request_msg, "request_logical_camera");

  std::string serialized;
  request_msg.SerializeToString(&serialized);

  const auto reply = info_bridge_ptr->RequestReply(serialized);

  cloisim::msgs::LogicalCameraSensor sensor_info;
  if (reply.size() > 0 && sensor_info.ParseFromString(reply)) {
    const auto hfov = sensor_info.horizontal_fov();
    const auto aspect_ratio = sensor_info.aspect_ratio();
    const auto near_clip = sensor_info.near_clip();
    const auto far_clip = sensor_info.far_clip();

    LOG_I(this, "LogicalCameraSensor: hfov=" << hfov
      << " aspect_ratio=" << aspect_ratio
      << " near=" << near_clip << " far=" << far_clip);

    msg_camera_info_.header.frame_id = msg_.header.frame_id;
    msg_camera_info_.width = 0;
    msg_camera_info_.height = 0;
    msg_camera_info_.distortion_model = "none";

    // Store horizontal_fov in K[0] (fx) and aspect_ratio-derived vertical fov in K[4] (fy)
    // near_clip in P[3], far_clip in P[7] as auxiliary info
    const auto fx = 1.0 / (2.0 * tan(hfov / 2.0));
    const auto fy = fx * aspect_ratio;

    msg_camera_info_.k.fill(0.0);
    msg_camera_info_.k[0] = fx;
    msg_camera_info_.k[4] = fy;
    msg_camera_info_.k[8] = 1.0;

    msg_camera_info_.r.fill(0.0);
    msg_camera_info_.r[0] = 1.0;
    msg_camera_info_.r[4] = 1.0;
    msg_camera_info_.r[8] = 1.0;

    msg_camera_info_.p.fill(0.0);
    msg_camera_info_.p[0] = fx;
    msg_camera_info_.p[3] = near_clip;
    msg_camera_info_.p[5] = fy;
    msg_camera_info_.p[7] = far_clip;
    msg_camera_info_.p[10] = 1.0;
  } else {
    LOG_E(this, "Failed to get LogicalCameraSensor info");
  }
}

}  // namespace cloisim_ros
