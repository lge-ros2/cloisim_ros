/**
 *  @file   cloisim_ros_base.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CLOiSim-ROS base base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_base/base.hpp"
#include "cloisim_ros_base/helper.h"

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;

Base::Base(const string node_name_, const rclcpp::NodeOptions &options_, const int number_of_bridges)
    : Base(node_name_, "",  rclcpp::NodeOptions(options_), number_of_bridges)
{
}

Base::Base(const string node_name_, const int number_of_bridges)
    : Base(node_name_, "", rclcpp::NodeOptions(), number_of_bridges)
{
}

Base::Base(const string node_name_, const string namespace_, const int number_of_bridges)
    : Base(node_name_, namespace_,  rclcpp::NodeOptions(), number_of_bridges)
{
}

Base::Base(const string node_name_, const string namespace_, const rclcpp::NodeOptions &options_, const int number_of_bridges)
    : Node(node_name_,
           namespace_,
           rclcpp::NodeOptions(options_)
               .automatically_declare_parameters_from_overrides(true)
               .append_parameter_override("use_sim_time", true)
               .arguments(vector<string>{"--ros-args", "--remap", "/tf:=tf", "--remap", "/tf_static:=tf_static"}))
    , m_bRunThread(false)
    , m_node_handle(shared_ptr<rclcpp::Node>(this, [](auto) {}))
{
  SetupBridges(number_of_bridges);
}

Base::~Base()
{
  // DBG_SIM_INFO("Delete");
  for (auto pBridge : m_bridgeList)
  {
    delete pBridge;
  }
}

void Base::SetupBridges(const int number_of_bridges)
{
  m_bridgeList.reserve(number_of_bridges);

  for (size_t index = 0; index < m_bridgeList.capacity(); index++)
  {
    auto pBridge = new zmq::Bridge();
    m_bridgeList.push_back(pBridge);
  }
}

void Base::Start(const bool runSingleDataThread)
{
  m_bRunThread = true;

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node_handle);
  m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node_handle);

  Initialize();

  if (runSingleDataThread)
  {
    m_thread = thread([=]() {
      while (IsRunThread()) {
        UpdateData();
      }});
  }

  auto callback_static_tf_pub = [this]() -> void {
    PublishStaticTF();
  };

  // ROS2 timer for static tf
  m_timer = this->create_wall_timer(0.5s, callback_static_tf_pub);
}

void Base::Stop()
{
  m_bRunThread = false;

  if (m_thread.joinable())
  {
    m_thread.join();
    // DBG_SIM_INFO("Thread finished");
  }

  Deinitialize();

  CloseBridges();
}

void Base::SetupStaticTf2Message(const cloisim::msgs::Pose transform, const string child_frame_id, const string parent_frame_id)
{
  geometry_msgs::msg::TransformStamped msg_tf;
  msg_tf.header.frame_id = parent_frame_id;
  msg_tf.child_frame_id = child_frame_id;
  SetVector3MessageToGeometry(transform.position(), msg_tf.transform.translation);
  SetQuaternionMessageToGeometry(transform.orientation(), msg_tf.transform.rotation);

  AddStaticTf2(msg_tf);
}

void Base::PublishTF()
{
  m_tf_broadcaster->sendTransform(m_tf_list);
  m_tf_list.clear();
}

void Base::PublishStaticTF()
{
  // Update timestamp
  for (auto &_tf : m_static_tf_list)
  {
    _tf.header.stamp = m_simTime;
  }

  m_static_tf_broadcaster->sendTransform(m_static_tf_list);
}

zmq::Bridge* Base::GetBridge(const uint bridge_index)
{
  if (bridge_index >= m_bridgeList.capacity())
  {
    DBG_SIM_WRN("Wrong bridge index(%d) / total sim bridges(%lu)", bridge_index, m_bridgeList.capacity());
    return nullptr;
  }

  return m_bridgeList.at(bridge_index);
}

void Base::CloseBridges()
{
  for (auto pBridge : m_bridgeList)
  {
    pBridge->Disconnect();
  }
}

string Base::GetRobotName()
{
  bool isSingleMode;
  get_parameter("single_mode", isSingleMode);

  string robotName;
  get_parameter("single_mode.robotname", robotName);

  return (isSingleMode) ? robotName : string(get_namespace()).substr(1);
}

msgs::Pose Base::GetObjectTransform(const int bridge_index, const string target_name)
{
  auto const pBridge = GetBridge(bridge_index);
  return GetObjectTransform(pBridge, target_name);
}

msgs::Pose Base::GetObjectTransform(zmq::Bridge* const pBridge, const string target_name)
{
  msgs::Pose transform;
  transform.Clear();

  if (pBridge == nullptr)
  {
    return transform;
  }

  const auto reply = RequestReplyMessage(pBridge, "request_transform", target_name);

  if (reply.ByteSize() <= 0)
  {
    DBG_SIM_ERR("Faild to get object transform, length(%ld)", reply.ByteSize());
  }
  else
  {
    if (reply.IsInitialized() &&
        reply.name() == "transform" &&
        reply.has_value())
    {
      transform.CopyFrom(reply.value().pose3d_value());
    }
  }

  return transform;
}

void Base::GetRos2Parameter(zmq::Bridge* const pBridge)
{
  if (pBridge == nullptr)
  {
    return;
  }

  const auto reply = RequestReplyMessage(pBridge, "request_ros2");

  if (reply.ByteSize() <= 0)
  {
    DBG_SIM_ERR("Faild to get ROS2 common info, length(%ld)", reply.ByteSize());
  }
  else
  {
    if (reply.IsInitialized() &&
        reply.name() == "ros2")
    {
      for (auto i = 0; i < reply.children_size(); i++)
      {
        const auto param = reply.children(i);
        const auto paramValue = (param.has_value() && param.value().type() == msgs::Any_ValueType_STRING) ? param.value().string_value() : "";

        if (param.name() == "topic_name")
        {
          topic_name_ = paramValue;
          if (!paramValue.empty())
          {
            DBG_SIM_INFO("topic_name: %s", topic_name_.c_str());
          }
        }
        else if (param.name() == "frame_id")
        {
          frame_id_list_.push_back(paramValue);
          DBG_SIM_INFO("frame_id: %s", paramValue.c_str());
        }
      }
    }

  }
}

bool Base::GetBufferFromSimulator(const uint bridge_index, void** ppBbuffer, int& bufferLength, const bool isNonBlockingMode)
{
  auto pBridge = GetBridge(bridge_index);
  if (pBridge == nullptr)
  {
    DBG_SIM_ERR("sim bridge is null!!");
    return false;
  }

  const auto succeeded = pBridge->Receive(ppBbuffer, bufferLength, isNonBlockingMode);
  if (!succeeded || bufferLength < 0)
  {
    return false;
  }

  return true;
}

void Base::SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const string child_frame_id, const string header_frame_id)
{
  SetTf2(target_msg, IdentityPose(), child_frame_id, header_frame_id);
}

void Base::SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const msgs::Pose transform, const string child_frame_id, const string header_frame_id)
{
  target_msg.header.frame_id = header_frame_id;
  target_msg.child_frame_id = child_frame_id;
  SetVector3MessageToGeometry(transform.position(), target_msg.transform.translation);
  SetQuaternionMessageToGeometry(transform.orientation(), target_msg.transform.rotation);
}

void Base::SetupStaticTf2(const string child_frame_id, const string header_frame_id)
{
  SetupStaticTf2(IdentityPose(), child_frame_id, header_frame_id);
}

void Base::SetupStaticTf2(const msgs::Pose transform, const string child_frame_id, const string header_frame_id)
{
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = header_frame_id;
  static_tf.child_frame_id = child_frame_id;
  SetVector3MessageToGeometry(transform.position(), static_tf.transform.translation);
  SetQuaternionMessageToGeometry(transform.orientation(), static_tf.transform.rotation);

  AddStaticTf2(static_tf);
}

msgs::Param Base::RequestReplyMessage(zmq::Bridge* const pBridge, const string request_message, const string request_value)
{
  msgs::Param reply;

  string serialized_request_data;
  msgs::Param request;
  request.set_name(request_message);

  if (!request_value.empty())
  {
    auto pVal = request.mutable_value();
    pVal->set_type(cloisim::msgs::Any::STRING);
    pVal->set_string_value(request_value);
  }

  request.SerializeToString(&serialized_request_data);

  const auto serialized_reply_data = pBridge->RequestReply(serialized_request_data);

  if (serialized_reply_data.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get reply data, length(%ld)", serialized_reply_data.size());
  }
  else
  {
    if (reply.ParseFromString(serialized_reply_data) == false)
    {
      DBG_SIM_ERR("Faild to parse serialized buffer, pBuffer(%p) length(%ld)", serialized_reply_data.data(), serialized_reply_data.size());
    }
  }

  return reply;
}

msgs::Pose Base::IdentityPose()
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

string Base::GetFrameId(const string default_frame_id)
{
  return (frame_id_list_.size() == 0) ? default_frame_id : frame_id_list_.back();
}