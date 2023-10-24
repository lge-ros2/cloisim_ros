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
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/transform_stamped.pb.h>
#include <cloisim_msgs/twist.pb.h>
#include <cloisim_ros_base/helper.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace chrono_literals;
using namespace std::placeholders;
using namespace cloisim;

namespace cloisim_ros
{

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
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));
  get_parameter_or("bridge.Tx", portTx, uint16_t(0));
  get_parameter_or("bridge.Rx", portRx, uint16_t(0));
  get_parameter_or("bridge.Tf", portTf, uint16_t(0));

  const auto hashKeyInfo = GetTargetHashKey("Info");
  const auto hashKeyPub = GetTargetHashKey("Rx");
  const auto hashKeySub = GetTargetHashKey("Tx");
  const auto hashKeyTf = GetTargetHashKey("Tf");
  DBG_SIM_INFO("hashKey: pub(%s) sub(%s) tf(%s)", hashKeyPub.c_str(), hashKeySub.c_str(), hashKeyTf.c_str());

  auto info_bridge_ptr = CreateBridge();
  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetStaticTransforms(info_bridge_ptr);

    GetRobotDescription(info_bridge_ptr);
  }

  auto data_bridge_ptr = CreateBridge();
  if (data_bridge_ptr != nullptr)
  {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::PUB, portRx, hashKeyPub);
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portTx, hashKeySub);
    AddPublisherThread(data_bridge_ptr,
                       bind(&JointControl::PublishData, this, std::placeholders::_1));
  }

  auto tf_bridge_ptr = CreateBridge();
  if (tf_bridge_ptr != nullptr)
  {
    tf_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portTf, hashKeyTf);
    AddPublisherThread(tf_bridge_ptr,
                       bind(&Base::GenerateTF, this, std::placeholders::_1));
  }

  auto callback_sub = [this, data_bridge_ptr](
                          const control_msgs::msg::JointJog::SharedPtr msg) -> void
  {
    // const auto duration = msg->duration;
    const auto use_displacement = (msg->joint_names.size() == msg->displacements.size());
    const auto use_velocity = (msg->joint_names.size() == msg->velocities.size());

    for (size_t i = 0; i < msg->joint_names.size(); i++)
    {
      const auto joint_name = msg->joint_names[i];
      const auto displacement = (use_displacement) ? msg->displacements[i] : 0;
      const auto velocity = (use_velocity) ? msg->velocities[i] : 0;

      // DBG_SIM_INFO("%s %f %f", joint_name.c_str(), displacement, velocity);
      const auto msgBuf = MakeCommandMessage(joint_name, use_displacement, use_velocity, displacement, velocity);
      SetBufferToSimulator(data_bridge_ptr, msgBuf);
      rclcpp::sleep_for(500us);
    }
  };

  // ROS2 Publisher
  pub_joint_state_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());

  // ROS2 Subscriber
  sub_joint_job_ = create_subscription<control_msgs::msg::JointJog>(
      "joint_command", rclcpp::SensorDataQoS(), callback_sub);

  pub_robot_desc_ = create_publisher<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local());

  pub_robot_desc_->publish(msg_description_);
}

string JointControl::MakeCommandMessage(
    const string joint_name,
    const bool use_displacement,
    const bool use_velocity,
    const double joint_displacement,
    const double joint_velocity) const
{
  msgs::JointCmd jointCmd;

  jointCmd.set_name(joint_name);

  if (use_displacement)
  {
    auto position = jointCmd.mutable_position();
    position->set_target(joint_displacement);
  }

  if (use_velocity)
  {
    auto velocity = jointCmd.mutable_velocity();
    velocity->set_target(joint_velocity);
  }

  string message;
  jointCmd.SerializeToString(&message);
  return message;
}

void JointControl::PublishData(const string &buffer)
{
  cloisim::msgs::JointState_V pb_joint_states;
  if (!pb_joint_states.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_joint_states.header().stamp());

  sensor_msgs::msg::JointState msg_jointstate;
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

void JointControl::GetRobotDescription(zmq::Bridge *const bridge_ptr)
{
  if (bridge_ptr == nullptr)
  {
    return;
  }

  const auto reply = RequestReplyMessage(bridge_ptr, "robot_description");

  if (reply.IsInitialized() &&
      (reply.name().compare("description") == 0))
  {
    if (reply.value().type() == msgs::Any_ValueType_STRING)
    {
      const auto description = reply.value().string_value();
      msg_description_.data = description;
    }
  }
}

}  // namespace cloisim_ros
