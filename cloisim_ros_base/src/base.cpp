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
#include <cloisim_msgs/transform_stamped.pb.h>

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;

Base::Base(const string node_name, const rclcpp::NodeOptions &options)
    : Base(node_name, "",  rclcpp::NodeOptions(options))
{
}

Base::Base(const string node_name)
    : Base(node_name, "", rclcpp::NodeOptions())
{
}

Base::Base(const string node_name, const string namespace_)
    : Base(node_name, namespace_,  rclcpp::NodeOptions())
{
}

Base::Base(const string node_name, const string namespace_, const rclcpp::NodeOptions &options)
    : Node(node_name,
           namespace_,
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)
               .append_parameter_override("use_sim_time", true)
               .arguments(vector<string>{"--ros-args", "--remap", "/tf:=tf", "--remap", "/tf_static:=tf_static"}))
    , m_bRunThread(false)
    , m_node_handle(shared_ptr<rclcpp::Node>(this, [](auto) {}))
    , m_static_tf_broadcaster(nullptr)
    , m_tf_broadcaster(nullptr)
{
}

Base::~Base()
{
  // DBG_SIM_INFO("Delete");
  for (auto item : m_created_bridges)
  {
    delete item;
  }
}

void Base::Start(const bool enable_tf_publish)
{
  const auto wallTimerPeriod = 0.5s;
  m_bRunThread = true;

  if (enable_tf_publish)
  {
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node_handle);
    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node_handle);
  }

  Initialize();

  auto callback_static_tf_pub = [this]() -> void {
    PublishStaticTF();
  };

  // ROS2 timer for static tf
  m_timer = this->create_wall_timer(wallTimerPeriod, callback_static_tf_pub);
}

void Base::Stop()
{
  m_bRunThread = false;

  for (auto &thread : m_threads)
  {
    if (thread.joinable())
    {
      thread.join();
      // DBG_SIM_INFO("Thread finished");
    }
  }

  Deinitialize();

  CloseBridges();
}

void Base::GenerateTF(const string &buffer)
{
  cloisim::msgs::TransformStamped pb_transform_stamped;
  if (!pb_transform_stamped.ParseFromString(buffer))
  {
    DBG_SIM_ERR("%s: Parsing error, size(%d)", get_name(), buffer.length());
    return;
  }

  if (pb_transform_stamped.header().has_str_id() && pb_transform_stamped.transform().has_name())
  {
    geometry_msgs::msg::TransformStamped newTf;
    newTf.header.stamp = Convert(pb_transform_stamped.header().stamp());
    newTf.header.frame_id = pb_transform_stamped.header().str_id();
    newTf.child_frame_id = pb_transform_stamped.transform().name();
    // DBG_SIM_INFO("%ld %ld %s %s", newTf.header.stamp.sec, newTf.header.stamp.nanosec, newTf.header.frame_id.c_str(), newTf.child_frame_id.c_str());
    SetTf2(newTf, pb_transform_stamped.transform(), pb_transform_stamped.transform().name(), pb_transform_stamped.header().str_id());
    PublishTF(newTf);
  }
  else
  {
    DBG_SIM_WRN("empty child frame id or parent frame id");
  }
}

void Base::PublishTF(const geometry_msgs::msg::TransformStamped& tf)
{
  m_tf_broadcaster->sendTransform(tf);
}

void Base::PublishTF()
{
  if (m_tf_broadcaster != nullptr && m_tf_list.size() > 0)
  {
    m_tf_broadcaster->sendTransform(m_tf_list);
    m_tf_list.clear();
  }
}

void Base::PublishStaticTF()
{
  // Update timestamp
  for (auto &_tf : m_static_tf_list)
  {
    _tf.header.stamp = GetTime();
  }

  if (m_static_tf_broadcaster != nullptr && m_static_tf_list.size() > 0)
  {
    m_static_tf_broadcaster->sendTransform(m_static_tf_list);
  }
}

zmq::Bridge* Base::CreateBridge()
{
  const auto bridge_ptr = new zmq::Bridge();
  m_created_bridges.emplace_back(bridge_ptr);

  return bridge_ptr;
}

void Base::CloseBridges()
{
  for (auto item : m_created_bridges)
  {
    item->Disconnect();
  }
}

void Base::AddPublisherThread(zmq::Bridge *const bridge_ptr, function<void(const string &)> thread_func)
{
  m_threads.emplace_back(thread(
      [this, bridge_ptr, thread_func]()
      {
        while (IsRunThread())
        {
          void *buffer_ptr = nullptr;
          int bufferLength = 0;
          const bool succeeded = GetBufferFromSimulator(bridge_ptr, &buffer_ptr, bufferLength);
          if (!succeeded || bufferLength < 0)
          {
            continue;
          }

          const string buffer((const char *)buffer_ptr, bufferLength);
          thread_func(buffer);
        }
      }));
}

string Base::GetModelName()
{
  string model_name;
  get_parameter_or("model", model_name, string(""));
  return model_name;
}

string Base::GetRobotName()
{
  bool is_single_mode;
  get_parameter("single_mode", is_single_mode);

  string robotName;
  get_parameter("single_mode.robotname", robotName);

  return (is_single_mode) ? robotName : string(get_namespace()).substr(1);
}

msgs::Pose Base::GetObjectTransform(zmq::Bridge* const bridge_ptr, const string target_name, string& parent_frame_id)
{
  msgs::Pose transform;
  transform.Clear();

  if (bridge_ptr == nullptr)
  {
    return transform;
  }

  const auto reply = RequestReplyMessage(bridge_ptr, "request_transform", target_name);

  if (reply.ByteSize() <= 0)
  {
    DBG_SIM_ERR("Faild to get object transform, length(%ld)", reply.ByteSize());
  }
  else
  {
    if (reply.IsInitialized() &&
        reply.name() == "transform" && reply.has_value())
    {
      transform.CopyFrom(reply.value().pose3d_value());
      // DBG_SIM_INFO("transform receivied : %s", transform.name().c_str());

      if (reply.children_size() > 0)
      {
        const auto child_param = reply.children(0);
        if (child_param.name() == "parent_frame_id" && child_param.has_value() &&
            child_param.value().type() == msgs::Any_ValueType_STRING &&
            !child_param.value().string_value().empty())
        {
          // set parent_frame_id into name if exists.
          parent_frame_id = child_param.value().string_value();
        }
      }
    }
  }

  return transform;
}

void Base::GetRos2Parameter(zmq::Bridge* const bridge_ptr)
{
  if (bridge_ptr == nullptr)
  {
    return;
  }

  const auto reply = RequestReplyMessage(bridge_ptr, "request_ros2");

  if (reply.ByteSize() <= 0)
  {
    DBG_SIM_ERR("Faild to get ROS2 common info, length(%ld)", reply.ByteSize());
  }
  else
  {
    if (reply.IsInitialized() && reply.name() == "ros2")
    {
      for (auto i = 0; i < reply.children_size(); i++)
      {
        const auto param = reply.children(i);
        const auto paramValue = (param.has_value() && param.value().type() == msgs::Any_ValueType_STRING) ? param.value().string_value() : "";

        if (param.name().compare("topic_name") == 0)
        {
          topic_name_ = paramValue;
          if (!paramValue.empty())
          {
            DBG_SIM_INFO("topic_name: %s", topic_name_.c_str());
          }
        }
        else if (param.name().compare("frame_id") == 0)
        {
          frame_id_list_.push_back(paramValue);
          DBG_SIM_INFO("frame_id: %s", paramValue.c_str());
        }
      }
    }
  }
}

bool Base::GetBufferFromSimulator(zmq::Bridge* const bridge_ptr, void** ppBbuffer, int& bufferLength, const bool isNonBlockingMode)
{
  if (bridge_ptr == nullptr)
  {
    DBG_SIM_ERR("sim bridge is null!!");
    return false;
  }

  const auto succeeded = bridge_ptr->Receive(ppBbuffer, bufferLength, isNonBlockingMode);
  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Bridge Ptr: %p", bridge_ptr);
    return false;
  }

  return true;
}

bool Base::SetBufferToSimulator(zmq::Bridge* const bridge_ptr, const string &buffer)
{
  if (!buffer.empty() && buffer.size() > 0 && bridge_ptr != nullptr)
  {
    return bridge_ptr->Send(buffer.data(), buffer.size());
  }

  return false;
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

void Base::SetStaticTf2(const string child_frame_id, const string header_frame_id)
{
  auto emptyPose = IdentityPose();
  emptyPose.set_name(child_frame_id);
  SetStaticTf2(emptyPose, header_frame_id);
}

void Base::SetStaticTf2(const msgs::Pose transform, const string parent_header_frame_id)
{
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = parent_header_frame_id;
  static_tf.child_frame_id = transform.name();
  SetVector3MessageToGeometry(transform.position(), static_tf.transform.translation);
  SetQuaternionMessageToGeometry(transform.orientation(), static_tf.transform.rotation);

  AddStaticTf2(static_tf);
}

msgs::Param Base::RequestReplyMessage(zmq::Bridge* const bridge_ptr, const string request_message, const string request_value)
{
  msgs::Param reply;

  if (bridge_ptr != nullptr)
  {
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

    const auto serialized_reply_data = bridge_ptr->RequestReply(serialized_request_data);

    if (serialized_reply_data.size() <= 0)
    {
      DBG_SIM_ERR("Faild to get reply data, length(%ld)", serialized_reply_data.size());
    }
    else
    {
      if (reply.ParseFromString(serialized_reply_data) == false)
      {
        DBG_SIM_ERR("Faild to parse serialized buffer, buffer_ptr(%p) length(%ld)", serialized_reply_data.data(), serialized_reply_data.size());
      }
    }
  }

  return reply;
}

msgs::Param Base::RequestReplyMessage(zmq::Bridge* const bridge_ptr, const msgs::Param request_message)
{
  msgs::Param response_msg;

  string serializedBuffer;
  request_message.SerializeToString(&serializedBuffer);

  if (bridge_ptr != nullptr)
  {
    const auto reply_data = bridge_ptr->RequestReply(serializedBuffer);
    response_msg.ParseFromString(reply_data);
  }

  return response_msg;
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

void Base::SetTime(const cloisim::msgs::Time &time)
{
  SetTime(time.sec(), time.nsec());
}

void Base::SetTime(const int32_t seconds, const uint32_t nanoseconds)
{
  m_sim_time = Convert(seconds, nanoseconds);
}