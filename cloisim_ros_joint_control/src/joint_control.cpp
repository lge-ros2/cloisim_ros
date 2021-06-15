/**
 *  @file   joint_control.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 JointControl class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_joint_control/joint_control.hpp"
#include <cloisim_ros_base/helper.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/twist.pb.h>


using namespace std;
using namespace chrono_literals;
using namespace placeholders;
using namespace cloisim;
using namespace cloisim_ros;

JointControl::JointControl(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
{
  Start();
}

JointControl::JointControl(const string namespace_)
    : JointControl(rclcpp::NodeOptions(), "cloisim_ros_joint_control", namespace_)
{
}

JointControl::~JointControl()
{
  Stop();
}

void JointControl::Initialize()
{
  uint16_t portInfo, portTx, portRx;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));
  get_parameter_or("bridge.Tx", portTx, uint16_t(0));
  get_parameter_or("bridge.Rx", portRx, uint16_t(0));

  const auto hashKeyInfo = GetTargetHashKey("Info");
  const auto hashKeyPub = GetTargetHashKey("Rx");
  const auto hashKeySub = GetTargetHashKey("Tx");
  DBG_SIM_INFO("hash Key: info(%s) pub_(%s) sub(%s)", hashKeyInfo.c_str(), hashKeyPub.c_str(), hashKeySub.c_str());

  auto pBridgeData = CreateBridge(hashKeyPub);
  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::PUB, portRx, hashKeyPub);
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portTx, hashKeySub);
    CreatePublisherThread(pBridgeData);
  }

  auto callback_sub = [this, pBridgeData](const control_msgs::msg::JointJog::SharedPtr msg) -> void {
    // const auto duration = msg->duration;
    for (size_t i = 0; i < msg->joint_names.size(); i++)
    {
      const auto joint_name = msg->joint_names[i];
      const auto displacement = msg->displacements[i];
      const auto velocity = msg->velocities[i];

      const auto msgBuf = MakeCommandMessage(joint_name, displacement, velocity);
      SetBufferToSimulator(pBridgeData, msgBuf);
    }
  };

  // ROS2 Publisher
  pub_joint_state_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());

  // ROS2 Subscriber
  sub_joint_job_ = create_subscription<control_msgs::msg::JointJog>("joint_command", rclcpp::SensorDataQoS(), callback_sub);
}

string JointControl::MakeCommandMessage(const string joint_name, const double joint_displacement, const double joint_velocity) const
{
  msgs::JointCmd jointCmd;

  jointCmd.set_name(joint_name);

  auto position = jointCmd.mutable_position();
  position->set_target(joint_displacement);

  auto velocity = jointCmd.mutable_velocity();
  velocity->set_target(joint_velocity);

  string message;
  jointCmd.SerializeToString(&message);
  return message;
}

void JointControl::UpdatePublishingData(const string &buffer)
{
  if (!pb_joint_states.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_joint_states.header().stamp());

  msg_jointstate_.header.stamp = GetTime();
  msg_jointstate_.name.clear();
  msg_jointstate_.effort.clear();
  msg_jointstate_.position.clear();
  msg_jointstate_.velocity.clear();

  for (auto i = 0; i < pb_joint_states.jointstate_size(); i++)
  {
    const auto joint_state = pb_joint_states.jointstate(i);

    msg_jointstate_.name.push_back(joint_state.name());
    msg_jointstate_.effort.push_back(joint_state.effort());
    msg_jointstate_.position.push_back(joint_state.position());
    msg_jointstate_.velocity.push_back(joint_state.velocity());
  }

  // publish data
  pub_joint_state_->publish(msg_jointstate_);

  PublishTF();
}