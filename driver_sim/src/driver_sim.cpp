/**
 *  @file   driver_sim.cpp
 *  @date   2020-05-22
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 driver Sim base base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <driver_sim/driver_sim.hpp>

using namespace std;
using namespace gazebo;

DriverSim::DriverSim(const string node_name, const int number_of_simbridge)
    : Node(node_name,
           rclcpp::NodeOptions()
               .parameter_overrides(vector<rclcpp::Parameter>{rclcpp::Parameter("use_sim_time", true)})
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
    , m_bRunThread(false)
    , m_node_handle(shared_ptr<rclcpp::Node>(this, [](auto) {}))
{
  m_simBridgeList.reserve(number_of_simbridge);

  for (auto index = 0; index < number_of_simbridge; index++)
  {
    auto pSimBridge = new SimBridge();
    m_simBridgeList.push_back(pSimBridge);
  }

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node_handle);
  m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node_handle);
}

DriverSim::~DriverSim()
{
  // DBG_SIM_INFO("Delete");
  for (auto pSimBridge : m_simBridgeList)
  {
    delete pSimBridge;
  }
}

void DriverSim::Start(const bool runSingleDataThread)
{
  m_bRunThread = true;

  Initialize();

  if (runSingleDataThread)
  {
    m_thread = thread([=]() {
      while (IsRunThread()) {
        UpdateData();
      }});
  }

  auto callback_pub = [this]() -> void {
    PublishStaticTF();
  };

  // ROS2 timer for Publisher
  m_timer = this->create_wall_timer(0.5s, callback_pub);
}

void DriverSim::Stop()
{
  m_bRunThread = false;

  usleep(200);

  if (m_thread.joinable())
  {
    m_thread.join();
    // DBG_SIM_INFO("Thread finished");
  }

  Deinitialize();
}

void DriverSim::SetupStaticTf2Message(const gazebo::msgs::Pose transform, const string child_frame_id, const string parent_frame_id)
{
  geometry_msgs::msg::TransformStamped msg_tf;
  msg_tf.header.frame_id = parent_frame_id;
  msg_tf.child_frame_id = child_frame_id;
  msg_tf.transform.translation.x = transform.position().x();
  msg_tf.transform.translation.y = transform.position().y();
  msg_tf.transform.translation.z = transform.position().z();
  msg_tf.transform.rotation.x = transform.orientation().x();
  msg_tf.transform.rotation.y = transform.orientation().y();
  msg_tf.transform.rotation.z = transform.orientation().z();
  msg_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(msg_tf);
}

void DriverSim::PublishTF()
{
  m_tf_broadcaster->sendTransform(m_tf_list);
  m_tf_list.clear();
}

void DriverSim::PublishStaticTF()
{
  // Update timestamp
  for (auto _tf : m_static_tf_list)
  {
    _tf.header.stamp = m_simTime;
  }

  m_static_tf_broadcaster->sendTransform(m_static_tf_list);
}

SimBridge* DriverSim::GetSimBridge(const uint bridge_index)
{
  if (bridge_index >= m_simBridgeList.capacity())
  {
    DBG_SIM_WRN("Wrong bridge index(%d) / total sim bridges(%lu)", bridge_index, m_simBridgeList.capacity());
    return nullptr;
  }

  return m_simBridgeList.at(bridge_index);
}

void DriverSim::DisconnectSimBridges()
{
  for (auto pSimBridge : m_simBridgeList)
  {
    pSimBridge->Disconnect();
  }
}

msgs::Pose DriverSim::GetObjectTransform(const int bridge_index, const string target_name)
{
  auto const pSimBridge = GetSimBridge(bridge_index);
  return GetObjectTransform(pSimBridge, target_name);
}

msgs::Pose DriverSim::GetObjectTransform(SimBridge* const pSimBridge, const string target_name)
{
  msgs::Pose transform;
  transform.Clear();

  if (pSimBridge == nullptr)
  {
    return transform;
  }

  msgs::Param request_msg;
  request_msg.set_name("request_transform");

  auto pVal = request_msg.mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(target_name);

  string request;
  request_msg.SerializeToString(&request);

  const auto reply = pSimBridge->RequestReply(request);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get object transform, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param reply_msg;
    if (reply_msg.ParseFromString(reply))
    {
      if (reply_msg.IsInitialized() &&
          reply_msg.name() == "transform" &&
          reply_msg.has_value())
      {
        transform.CopyFrom(reply_msg.value().pose3d_value());
      }
    }
    else
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%ld)", reply.data(), reply.size());
    }
  }

  return transform;
}

void DriverSim::GetRos2Parameter(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  void *pBuffer = nullptr;
  int bufferLength = 0;

  request_msg.set_name("request_ros2");
  request_msg.SerializeToString(&serializedBuffer);

  pSimBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  const auto succeeded = pSimBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get ROS2 common info, length(%d)", bufferLength);
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }

    if (m_pbBufParam.IsInitialized() &&
        m_pbBufParam.name() == "ros2")
    {
      auto param0 = m_pbBufParam.children(0);
      if (param0.name() == "topic_name" && param0.has_value() &&
          param0.value().type() == msgs::Any_ValueType_STRING &&
          !param0.value().string_value().empty())
      {
        topic_name_ = param0.value().string_value();
      }

      auto param1 = m_pbBufParam.children(1);
      if (param1.name() == "frame_id" && param1.has_value() &&
          param1.value().type() == msgs::Any_ValueType_STRING &&
          !param1.value().string_value().empty())
      {
        frame_id_ = param1.value().string_value();
      }
    }

    DBG_SIM_INFO("topic_name: %s, frame_id: %s", topic_name_.c_str(), frame_id_.c_str());
  }
}

bool DriverSim::GetBufferFromSimulator(const uint bridge_index, void** ppBbuffer, int& bufferLength, const bool isNonBlockingMode)
{
  auto simBridge = GetSimBridge(bridge_index);
  if (simBridge == nullptr)
  {
    DBG_SIM_ERR("sim bridge is null!!");
    return false;
  }

  const auto succeeded = simBridge->Receive(ppBbuffer, bufferLength, false);
  if (!succeeded || bufferLength < 0)
  {
    return false;
  }

  return true;
}

void DriverSim::SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const string child_frame_id, const string header_frame_id)
{
  SetTf2(target_msg, IdentityPose(), child_frame_id, header_frame_id);
}

void DriverSim::SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const msgs::Pose transform, const string child_frame_id, const string header_frame_id)
{
  target_msg.header.frame_id = header_frame_id;
  target_msg.child_frame_id = child_frame_id;
  target_msg.transform.translation.x = transform.position().x();
  target_msg.transform.translation.y = transform.position().y();
  target_msg.transform.translation.z = transform.position().z();
  target_msg.transform.rotation.x = transform.orientation().x();
  target_msg.transform.rotation.y = transform.orientation().y();
  target_msg.transform.rotation.z = transform.orientation().z();
  target_msg.transform.rotation.w = transform.orientation().w();
}

void DriverSim::SetupStaticTf2(const string child_frame_id, const string header_frame_id)
{
  SetupStaticTf2(IdentityPose(), child_frame_id, header_frame_id);
}

void DriverSim::SetupStaticTf2(const msgs::Pose transform, const string child_frame_id, const string header_frame_id)
{
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = header_frame_id;
  static_tf.child_frame_id = child_frame_id;
  static_tf.transform.translation.x = transform.position().x();
  static_tf.transform.translation.y = transform.position().y();
  static_tf.transform.translation.z = transform.position().z();
  static_tf.transform.rotation.x = transform.orientation().x();
  static_tf.transform.rotation.y = transform.orientation().y();
  static_tf.transform.rotation.z = transform.orientation().z();
  static_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(static_tf);
}

gazebo::msgs::Param DriverSim::RequestReplyMessage(SimBridge* const pSimBridge, const gazebo::msgs::Param request_message)
{
  msgs::Param reply_message;

  string serialized_request_data;
  request_message.SerializeToString(&serialized_request_data);

  const auto serialized_reply_data = pSimBridge->RequestReply(serialized_request_data);

  if (serialized_reply_data.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get reply data, length(%ld)", serialized_reply_data.size());
  }
  else
  {
    if (reply_message.ParseFromString(serialized_reply_data) == false)
    {
      DBG_SIM_ERR("Faild to parse serialized buffer, pBuffer(%p) length(%ld)", serialized_reply_data.data(), serialized_reply_data.size());
    }
  }

  return reply_message;
}

gazebo::msgs::Pose DriverSim::IdentityPose()
{
  msgs::Pose identityTransform;
  identityTransform.mutable_position()->set_x(0.0);
  identityTransform.mutable_position()->set_y(0.0);
  identityTransform.mutable_position()->set_z(0.0);
  identityTransform.mutable_orientation()->set_x(0.0);
  identityTransform.mutable_orientation()->set_y(0.0);
  identityTransform.mutable_orientation()->set_z(0.0);
  identityTransform.mutable_orientation()->set_w(1.0);
  return identityTransform;
}