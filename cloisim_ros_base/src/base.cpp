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

#include <cloisim_ros_base/base.hpp>

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
  msg_tf.transform.translation.x = transform.position().x();
  msg_tf.transform.translation.y = transform.position().y();
  msg_tf.transform.translation.z = transform.position().z();
  msg_tf.transform.rotation.x = transform.orientation().x();
  msg_tf.transform.rotation.y = transform.orientation().y();
  msg_tf.transform.rotation.z = transform.orientation().z();
  msg_tf.transform.rotation.w = transform.orientation().w();

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

  msgs::Param request_msg;
  request_msg.set_name("request_transform");

  auto pVal = request_msg.mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(target_name);

  string request;
  request_msg.SerializeToString(&request);

  const auto reply = pBridge->RequestReply(request);

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

void Base::GetRos2Parameter(zmq::Bridge* const pBridge)
{
  if (pBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  void *pBuffer = nullptr;
  int bufferLength = 0;

  request_msg.set_name("request_ros2");
  request_msg.SerializeToString(&serializedBuffer);

  pBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  const auto succeeded = pBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get ROS2 common info, length(%d)", bufferLength);
  }
  else
  {
    msgs::Param pbParam;
    if (pbParam.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }

    if (pbParam.IsInitialized() &&
        pbParam.name() == "ros2")
    {
      auto param0 = pbParam.children(0);
      if (param0.name() == "topic_name" && param0.has_value() &&
          param0.value().type() == msgs::Any_ValueType_STRING &&
          !param0.value().string_value().empty())
      {
        topic_name_ = param0.value().string_value();
      }

      auto param1 = pbParam.children(1);
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
  target_msg.transform.translation.x = transform.position().x();
  target_msg.transform.translation.y = transform.position().y();
  target_msg.transform.translation.z = transform.position().z();
  target_msg.transform.rotation.x = transform.orientation().x();
  target_msg.transform.rotation.y = transform.orientation().y();
  target_msg.transform.rotation.z = transform.orientation().z();
  target_msg.transform.rotation.w = transform.orientation().w();
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
  static_tf.transform.translation.x = transform.position().x();
  static_tf.transform.translation.y = transform.position().y();
  static_tf.transform.translation.z = transform.position().z();
  static_tf.transform.rotation.x = transform.orientation().x();
  static_tf.transform.rotation.y = transform.orientation().y();
  static_tf.transform.rotation.z = transform.orientation().z();
  static_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(static_tf);
}

msgs::Param Base::RequestReplyMessage(zmq::Bridge* const pBridge, const string request_message)
{
  msgs::Param reply;

  string serialized_request_data;
  msgs::Param request;
  request.set_name(request_message);
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