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

  // SetTf2(odom_tf_, msg_odom_.child_frame_id, msg_odom_.header.frame_id);

  // info_bridge_ptr = CreateBridge(hashKeyInfo);
  // if (info_bridge_ptr != nullptr)
  // {
  //   info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

  //   GetTransformNameInfo(info_bridge_ptr);

  //   std::string base_link_name("base_link");
  //   SetStaticTf2(base_link_name, "base_footprint");

  //   const auto transform_imu_name = target_transform_name["imu"];
  //   const auto transform_imu = GetObjectTransform(info_bridge_ptr, transform_imu_name);
  //   SetStaticTf2(transform_imu, transform_imu_name + "_link", base_link_name);

  //   const auto transform_wheel_0_name = target_transform_name["wheels/left"];
  //   const auto transform_wheel_0 = GetObjectTransform(info_bridge_ptr, transform_wheel_0_name);
  //   SetTf2(wheel_left_tf_, transform_wheel_0, transform_wheel_0_name + "_link", base_link_name);

  //   const auto init_left_q_msg = &wheel_left_tf_.transform.rotation;
  //   const auto wheel_left_quat = tf2::Quaternion(init_left_q_msg->x, init_left_q_msg->y, init_left_q_msg->z, init_left_q_msg->w);
  //   tf2::Matrix3x3(wheel_left_quat).getRPY(orig_left_wheel_rot_[0], orig_left_wheel_rot_[1], orig_left_wheel_rot_[2]);

  //   const auto transform_wheel_1_name = target_transform_name["wheels/right"];
  //   const auto transform_wheel_1 = GetObjectTransform(info_bridge_ptr, transform_wheel_1_name);
  //   SetTf2(wheel_right_tf_, transform_wheel_1, transform_wheel_1_name + "_link", base_link_name);

  //   const auto init_right_q_msg = &wheel_right_tf_.transform.rotation;
  //   const auto wheel_right_quat = tf2::Quaternion(init_right_q_msg->x, init_right_q_msg->y, init_right_q_msg->z, init_right_q_msg->w);
  //   tf2::Matrix3x3(wheel_right_quat).getRPY(orig_right_wheel_rot_[0], orig_right_wheel_rot_[1], orig_right_wheel_rot_[2]);
  // }

  // ROS2 Publisher
  pub_joint_state_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());

  auto pBridgeData = CreateBridge(hashKeyPub);
  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::PUB, portRx, hashKeyPub);
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portTx, hashKeySub);
    CreatePublisherThread(pBridgeData);
  }
  auto callback_sub = [this, pBridgeData](const control_msgs::msg::JointJog::SharedPtr msg) -> void {
    const auto msgBuf = MakeCommandMessage(msg);
    SetBufferToSimulator(pBridgeData, msgBuf);
  };

  // ROS2 Subscriber
  sub_joint_job__ = create_subscription<control_msgs::msg::JointJog>("joint_command", rclcpp::SensorDataQoS(), callback_sub);
}

// void JointControl::GetTransformNameInfo(zmq::Bridge* const bridge_ptr)
// {
//   if (bridge_ptr == nullptr)
//   {
//     return;
//   }

//   const auto reply = RequestReplyMessage(bridge_ptr, "request_transform_name");

//   if (reply.IsInitialized() &&
//       (reply.name().compare("ros2") == 0))
//   {
//     auto baseParam = reply.children(0);
//     if (baseParam.IsInitialized() &&
//         baseParam.name() == "transform_name")
//     {
//       auto param0 = baseParam.children(0);
//       if (param0.name() == "imu" && param0.has_value() &&
//           param0.value().type() == msgs::Any_ValueType_STRING &&
//           !param0.value().string_value().empty())
//       {
//         target_transform_name["imu"] = param0.value().string_value();
//       }

//       auto param1 = baseParam.children(1);
//       if (param1.name() == "wheels")
//       {
//         auto childParam0 = param1.children(0);
//         if (childParam0.name() == "left" && childParam0.has_value() &&
//             childParam0.value().type() == msgs::Any_ValueType_STRING &&
//             !childParam0.value().string_value().empty())
//         {
//           target_transform_name["wheels/left"] = childParam0.value().string_value();
//         }

//         auto childParam1 = param1.children(1);
//         if (childParam1.name() == "right" && childParam1.has_value() &&
//             childParam1.value().type() == msgs::Any_ValueType_STRING &&
//             !childParam1.value().string_value().empty())
//         {
//           target_transform_name["wheels/right"] = childParam1.value().string_value();
//         }
//       }
//     }

//     DBG_SIM_INFO("transform name imu:%s, wheels(0/1):%s/%s",
//                  target_transform_name["imu"].c_str(),
//                  target_transform_name["wheels/left"].c_str(),
//                  target_transform_name["wheels/right"].c_str());
//   }
// }

string JointControl::MakeCommandMessage(const control_msgs::msg::JointJog::SharedPtr msg) const
{
  msgs::JointCmd jointCmd;

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

  SetSimTime(pb_joint_states.header().stamp());

  // PublishTF();

  // publish data
  pub_joint_state_->publish(msg_jointstate_);
}