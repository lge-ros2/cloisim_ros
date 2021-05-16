/**
 *  @file   ground_truth.cpp
 *  @date   2021-05-16
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 node for controlling unity simulation
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_groundtruth/ground_truth.hpp"
#include <cloisim_ros_base/helper.h>

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;

GroundTruth::GroundTruth(const rclcpp::NodeOptions &options_, const std::string node_name_)
  : Base(node_name_, options_, 1)
{
  Start();
}

GroundTruth::GroundTruth()
  : GroundTruth(rclcpp::NodeOptions(), "cloisim_ros_groundtruth")
{
}

GroundTruth::~GroundTruth()
{
  Stop();
}

void GroundTruth::Initialize()
{
  string model_name;
  uint16_t portData;
  get_parameter_or("model", model_name, string("GroundTruth"));
  get_parameter_or("bridge.Data", portData, uint16_t(0));

  hashKeySub_ = model_name + GetPartsName();
  DBG_SIM_INFO("hash Key sub: %s", hashKeySub_.c_str());

  auto pBridgeData = GetBridge(0);
  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeySub_ + "Data");
  }

  pub = create_publisher<perception_msgs::msg::ObjectArray>("/ground_truth", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
}

void GroundTruth::Deinitialize()
{
}

void GroundTruth::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  if (!pbBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBuf.header().stamp().sec(), pbBuf.header().stamp().nsec());

  UpdatePerceptionData();

  pub->publish(msg);
}

void GroundTruth::UpdatePerceptionData()
{
  msg.header.stamp = m_simTime;

  msg.objects.clear();

  for (auto i = 0; i < pbBuf.perception_size(); i++)
  {
    const auto perception = pbBuf.perception(i);

    auto object_pose_msg = perception_msgs::msg::ObjectPose();
    object_pose_msg.tracking_id = perception.tracking_id();
    object_pose_msg.class_id = perception.class_id();

    SetVector3MessageToGeometry(perception.position(), object_pose_msg.position);
    SetVector3MessageToGeometry(perception.velocity(), object_pose_msg.velocity);

    for (auto it = perception.footprints().begin(); it < perception.footprints().end(); ++it)
    {
      const auto point = *it;
      geometry_msgs::msg::Point32 point32;
      SetVector3MessageToGeometry(point, point32);
      object_pose_msg.footprints.push_back(point32);
    }

    msg.objects.push_back(object_pose_msg);
  }
}