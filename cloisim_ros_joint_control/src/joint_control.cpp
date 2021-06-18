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
#include <cloisim_msgs/transform_stamped.pb.h>


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
  uint16_t portInfo, portTx, portRx, portTf;
  // get_parameter_or("bridge.Info", portInfo, uint16_t(0));
  get_parameter_or("bridge.Tx", portTx, uint16_t(0));
  get_parameter_or("bridge.Rx", portRx, uint16_t(0));
  get_parameter_or("bridge.Tf", portTf, uint16_t(0));

  // const auto hashKeyInfo = GetTargetHashKey("Info");
  const auto hashKeyPub = GetTargetHashKey("Rx");
  const auto hashKeySub = GetTargetHashKey("Tx");
  const auto hashKeyTf = GetTargetHashKey("Tf");
  // DBG_SIM_INFO("hash Key: info(%s) pub_(%s) sub(%s) tf(%s)", hashKeyInfo.c_str(), hashKeyPub.c_str(), hashKeySub.c_str(), hashKeyTf.c_str());
  DBG_SIM_INFO("hash Key: pub(%s) sub(%s) tf(%s)", hashKeyPub.c_str(), hashKeySub.c_str(), hashKeyTf.c_str());

  auto pBridgeData = CreateBridge();
  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::PUB, portRx, hashKeyPub);
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portTx, hashKeySub);
    AddPublisherThread(pBridgeData, bind(&JointControl::PublishData, this, std::placeholders::_1));
  }

  auto pBridgeTf = CreateBridge();
  if (pBridgeTf != nullptr)
  {
    pBridgeTf->Connect(zmq::Bridge::Mode::SUB, portTf, hashKeyTf);
    AddPublisherThread(pBridgeTf, bind(&JointControl::GenerateTF, this, std::placeholders::_1));
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
  static msgs::JointCmd jointCmd;

  jointCmd.set_name(joint_name);

  auto position = jointCmd.mutable_position();
  position->set_target(joint_displacement);

  auto velocity = jointCmd.mutable_velocity();
  velocity->set_target(joint_velocity);

  static string message;
  jointCmd.SerializeToString(&message);
  return message;
}

void JointControl::GenerateTF(const string &buffer)
{
  static cloisim::msgs::TransformStamped pb_transform_stamped;
  if (!pb_transform_stamped.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  static geometry_msgs::msg::TransformStamped newTf;
  newTf.header.stamp = Convert(pb_transform_stamped.header().stamp());
  newTf.header.frame_id = pb_transform_stamped.header().str_id();
  newTf.child_frame_id = pb_transform_stamped.transform().name();
  // DBG_SIM_INFO("%ld %ld %s %s", newTf.header.stamp.sec, newTf.header.stamp.nanosec, newTf.header.frame_id.c_str(), newTf.child_frame_id.c_str());
  SetTf2(newTf, pb_transform_stamped.transform(), pb_transform_stamped.transform().name(), pb_transform_stamped.header().str_id());
  AddTf2(newTf);
  PublishTF();
}

void JointControl::PublishData(const string &buffer)
{
  static cloisim::msgs::JointState_V pb_joint_states;
  if (!pb_joint_states.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_joint_states.header().stamp());

  static sensor_msgs::msg::JointState msg_jointstate;
  msg_jointstate.header.stamp = GetTime();
  msg_jointstate.name.clear();
  msg_jointstate.effort.clear();
  msg_jointstate.position.clear();
  msg_jointstate.velocity.clear();

  for (auto i = 0; i < pb_joint_states.jointstate_size(); i++)
  {
    const auto joint_state = pb_joint_states.jointstate(i);

    msg_jointstate.name.push_back(joint_state.name());
    msg_jointstate.effort.push_back(joint_state.effort());
    msg_jointstate.position.push_back(joint_state.position());
    msg_jointstate.velocity.push_back(joint_state.velocity());
  }

  // publish data
  pub_joint_state_->publish(msg_jointstate);
}