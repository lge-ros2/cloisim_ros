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

#include <cloisim_msgs/pose.pb.h>

#include "cloisim_ros_base/base.hpp"
#include "cloisim_ros_base/param_helper.hpp"

using namespace std::literals::chrono_literals;
using string = std::string;

namespace cloisim_ros
{

Base::Base(const string node_name, const rclcpp::NodeOptions & options)
: Base(node_name, "", rclcpp::NodeOptions(options))
{
}

Base::Base(const string node_name)
: Base(node_name, "", rclcpp::NodeOptions()) {}

Base::Base(const string node_name, const string namespace_)
: Base(node_name, namespace_, rclcpp::NodeOptions())
{
}

Base::Base(const string node_name, const string namespace_, const rclcpp::NodeOptions & options)
: Node(
    node_name, namespace_,
    rclcpp::NodeOptions(options)
    .automatically_declare_parameters_from_overrides(true)
    .append_parameter_override("use_sim_time", true)
    .arguments(std::vector<string> {
    "--ros-args", "--remap", "/tf:=tf", "--remap", "/tf_static:=tf_static"
  }))
  , m_bRunThread(false)
  , m_node_handle(std::shared_ptr<rclcpp::Node>(this, [](auto) {}))
  , m_static_tf_broadcaster(nullptr)
  , m_tf_broadcaster(nullptr)
  , enable_tf_publish_(true)
{
  get_parameter_or("enable_tf", enable_tf_publish_, true);
}

Base::~Base()
{
  // LOG_I(this, "Delete");
  m_created_bridges.clear();
}

void Base::Start(const bool enable_tf_publish)
{
  const auto wallTimerPeriod = 0.5s;
  m_bRunThread = true;

  if (enable_tf_publish) {
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node_handle);
    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node_handle);
  }

  Initialize();

  LOG_I(this, "namespace(" << get_namespace() << ") node(" << get_name()
                           << ") enable_tf(" << enable_tf_publish << ")");

  auto callback_static_tf_pub = [this]() -> void {PublishStaticTF();};

  // ROS2 timer for static tf
  m_timer = this->create_wall_timer(wallTimerPeriod, callback_static_tf_pub);

  rclcpp::on_shutdown([&] {
      this->Stop();
  });
}

void Base::Stop()
{
  // LOG_I(this, "");
  auto expected = false;
  if (!m_stopping.compare_exchange_strong(expected, true)) {
    return;
  }

  m_bRunThread = false;

  if (m_timer) {
    m_timer->cancel(); // m_timer.reset();
  }

  CloseBridges();

  for (auto & thread : m_threads) {
    if (thread.joinable()) {
      thread.join(); // Thread finished
    }
  }

  Deinitialize();
}

void Base::GenerateTF(const void * buffer, int bufferLength)
{
  cloisim::msgs::Pose pb_pose;
  if (!pb_pose.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "[" << get_name() << "] Parsing error, size=" << bufferLength);
    return;
  }

  const auto parent_frame_id = param::GetHeaderDataValue(pb_pose.header(), "frame_id");
  if (!parent_frame_id.empty() && !pb_pose.name().empty()) {
    geometry_msgs::msg::TransformStamped newTf;
    newTf.header.stamp = msg::Convert(pb_pose.header().stamp());
    newTf.header.frame_id = parent_frame_id;
    newTf.child_frame_id = pb_pose.name();
    // LOG_I(this, newTf.header.stamp.sec << " " << newTf.header.stamp.nanosec << " " <<
    //              newTf.header.frame_id << " " << newTf.child_frame_id);
    SetTf2(newTf, pb_pose, pb_pose.name(), parent_frame_id);
    PublishTF(newTf);
#if 0
  } else {
    LOG_W(this, "empty child frame id or parent frame id");
#endif
  }
}

void Base::PublishTF()
{
  if (m_tf_broadcaster != nullptr && m_tf_list.size() > 0) {
    m_tf_broadcaster->sendTransform(m_tf_list);
    m_tf_list.clear();
  }
}

void Base::PublishStaticTF()
{
  // Update timestamp
  for (auto & _tf : m_static_tf_list) {
    _tf.header.stamp = GetTime();
  }

  if (m_static_tf_broadcaster != nullptr && m_static_tf_list.size() > 0) {
    m_static_tf_broadcaster->sendTransform(m_static_tf_list);
  }
}

zmq::Bridge * Base::CreateBridge()
{
  m_created_bridges.emplace_back(std::make_unique<zmq::Bridge>());
  return m_created_bridges.back().get();
}

void Base::CloseBridges()
{
  for (auto & bridge : m_created_bridges) {
    bridge->Disconnect();
  }
}

void Base::AddBridgeReceiveWorker(
  zmq::Bridge * const bridge_ptr, std::function<void(const void *, int)> data_process_func,
  const bool is_non_block)
{
  m_threads.emplace_back(
    [this, bridge_ptr, data_process_func, is_non_block]() {
      auto backoff_ms = 1;
      while (IsRunThread()) {
        void * buffer_ptr = nullptr;
        int bufferLength = 0;
        const bool succeeded = GetBufferFromSimulator(bridge_ptr, &buffer_ptr, bufferLength,
          is_non_block);
        const auto err = bridge_ptr->GetLastError();

        if (!succeeded || bufferLength < 0) {
          if (!IsRunThread()) {break;}
          if (err == EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            backoff_ms = std::min(backoff_ms * 2, backoff_max);
            if (backoff_ms == backoff_max) {
              const auto now = this->get_clock()->now();
              LOG_W(this,
                "[" << GetMainHashKey() <<
                "] t=" << std::fixed << std::setprecision(3) << now.seconds() <<
                " Timeout to get buffer(" << bufferLength << ") <= Sim, " <<
                bridge_ptr->GetErrorMessage(err));
            }
          } else if (err == ETERM) {
            break;
          } else {
            LOG_E(this,
              "[" << GetMainHashKey() << "] Failed to get buffer(" << bufferLength <<
              ") <= Sim, " << bridge_ptr->GetErrorMessage(err));
            std::this_thread::sleep_for(1ms);
          }
          continue;
        }

        backoff_ms = 1;

        if (!IsRunThread()) {break;}

        // Zero-copy: pass raw ZMQ buffer pointer directly to callback
        data_process_func(buffer_ptr, bufferLength);
      }
    });
}

void Base::AddBridgeServiceWorker(
  zmq::Bridge * const bridge_ptr,
  std::function<std::string(const std::string &)> service_process_func)
{
  m_threads.emplace_back(
    [this, bridge_ptr, service_process_func]() {
      auto backoff_ms = 1;
      while (IsRunThread()) {
        void * buffer_ptr = nullptr;
        int bufferLength = 0;
        const bool succeeded = GetBufferFromSimulator(bridge_ptr, &buffer_ptr, bufferLength, false);
        const auto err = bridge_ptr->GetLastError();
        if (!succeeded || bufferLength < 0) {
          if (!IsRunThread()) {break;}
          if (err == EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            backoff_ms = std::min(backoff_ms * 2, backoff_max);
            if (backoff_ms == backoff_max) {
              const auto now = this->get_clock()->now();
              LOG_W(this,
                "[" << GetMainHashKey() <<
                "] t=" << std::fixed << std::setprecision(3) << now.seconds() <<
                " Timeout to get buffer(" << bufferLength << ") <= Sim, " <<
                bridge_ptr->GetErrorMessage(err));
            }
          } else if (err == ETERM) {
            break;
          } else {
            LOG_E(this,
              "[" << GetMainHashKey() << "] Failed to get buffer(" << bufferLength <<
              ") <= Sim, " << bridge_ptr->GetErrorMessage(err));
            std::this_thread::sleep_for(1ms);
          }
          continue;
        }

        backoff_ms = 1;

        if (IsRunThread() == false) {break;}

        const std::string request_buffer((const char *)buffer_ptr, bufferLength);
        auto response_buffer = service_process_func(request_buffer);
        if (SetBufferToSimulator(bridge_ptr, response_buffer) == false) {
          LOG_E(this,
              "[" << GetMainHashKey() << "] Failed to set buffer(" << bufferLength <<
              ") => Sim, " << bridge_ptr->GetErrorMessage(err));
        }
      }
    });
}

string Base::GetModelName()
{
  string model_name;
  get_parameter_or("model", model_name, string(""));
  return model_name;
}

string Base::GetRobotName()
{
  bool is_single_mode = false;
  get_parameter("single_mode", is_single_mode);

  string robotName;
  get_parameter("single_mode.robotname", robotName);

  return (is_single_mode) ? robotName : string(get_namespace()).substr(1);
}

void Base::SetStaticTransforms(zmq::Bridge * const bridge_ptr)
{
  if (bridge_ptr == nullptr) {
    return;
  }

  const auto reply = RequestReplyMessage(bridge_ptr, "request_static_transforms");

  if (reply.IsInitialized() && param::HasKey(reply, "static_transforms")) {
    auto pose = cloisim::msgs::Pose();
    for (const auto & link : reply.children()) {
      if (link.IsInitialized() && param::HasValue(link)) {
        const auto link_name = param::GetName(link);
        const auto & link_value = param::GetValue(link);
        const auto parent_frame_id =
          (link_value.type() == cloisim::msgs::Any_ValueType_STRING &&
          link_name == "parent_frame_id") ?
          link_value.string_value() :
          "base_link";

        if (link.children_size() == 1) {
          const auto & child = link.children(0);
          const auto child_name = param::GetName(child);
          const auto & child_value = param::GetValue(child);

          if (param::HasValue(child)) {
            if (
              (child_name == "pose") &&
              child_value.type() == cloisim::msgs::Any_ValueType_POSE3D)
            {
              pose = child_value.pose3d_value();
            }
          }
        }

        SetStaticTf2(pose, parent_frame_id);
        LOG_I(this, "static transform " << pose.name() << " -> " << parent_frame_id);
      }
    }
  }
}

cloisim::msgs::Pose Base::GetObjectTransform(
  zmq::Bridge * const bridge_ptr, const string target_name, string & parent_frame_id)
{
  cloisim::msgs::Pose transform;
  transform.Clear();

  if (bridge_ptr == nullptr) {return transform;}

  const auto reply = RequestReplyMessage(bridge_ptr, "request_transform", target_name);

  if (reply.ByteSizeLong() > 0) {
    if (reply.IsInitialized() && param::HasKey(reply, "transform")) {
      const auto & transform_value = param::GetValue(reply, "transform");
      transform.CopyFrom(transform_value.pose3d_value());
      // LOG_I(this, "transform received=" << transform.name());

      if (reply.children_size() > 0) {
        const auto & child_param = reply.children(0);
        const auto child_name = param::GetName(child_param);
        const auto & child_value = param::GetValue(child_param);
        if (
          child_name == "parent_frame_id" && param::HasValue(child_param) &&
          child_value.type() == cloisim::msgs::Any_ValueType_STRING &&
          !child_value.string_value().empty())
        {
          // set parent_frame_id into name if exists.
          parent_frame_id = child_value.string_value();
        }
      }
    }
  } else {
    LOG_E(this, "Failed to get object transform, length=" << reply.ByteSizeLong());
  }

  return transform;
}

void Base::GetRos2Parameter(zmq::Bridge * const bridge_ptr)
{
  if (bridge_ptr == nullptr) {return;}

  const auto reply = RequestReplyMessage(bridge_ptr, "request_ros2");
  if (reply.ByteSizeLong() <= 0) {
    LOG_E(this, "Failed to get ROS2 common info, length=" << reply.ByteSizeLong());
  } else {
    if (reply.IsInitialized() && param::HasKey(reply, "ros2")) {
      for (auto i = 0; i < reply.children_size(); i++) {
        const auto & child = reply.children(i);
        const auto child_name = param::GetName(child);
        const auto & child_value = param::GetValue(child);
        const auto paramValue =
          (param::HasValue(child) &&
           child_value.type() == cloisim::msgs::Any_ValueType_STRING) ?
          child_value.string_value() : "";

        if (child_name == "topic_name") {
          topic_name_ = paramValue;
        } else if (child_name == "frame_id") {
          frame_id_list_.push_back(paramValue);
        }
      }

      std::string total_string;
      for (const auto & s : frame_id_list_) {
        total_string += (s + ", ");
      }
      LOG_I(this, "topic_name=" << topic_name_ << " frame_id=" << total_string);
    }
  }
}

bool Base::GetBufferFromSimulator(
  zmq::Bridge * const bridge_ptr, void ** ppBbuffer, int & bufferLength,
  const bool is_non_blocking_mode)
{
  if (bridge_ptr == nullptr) {
    LOG_E(nullptr, "Sim Bridge is NULL!!");
    return false;
  }

  const auto succeeded = bridge_ptr->Receive(ppBbuffer, bufferLength, is_non_blocking_mode);
  if (!succeeded || bufferLength < 0) {
    // LOG_E(this,
    //     "[" << GetMainHashKey() << "] Failed to receive buffer from Sim, len=" << bufferLength <<
    //     ", " << bridge_ptr->GetLastErrorMessage());
    return false;
  }

  return true;
}

void Base::SetTf2(
  geometry_msgs::msg::TransformStamped & target_msg, const cloisim::msgs::Pose transform,
  const string child_frame_id, const string header_frame_id)
{
  target_msg.header.frame_id = header_frame_id;
  target_msg.child_frame_id = child_frame_id;
  msg::Convert(transform.position(), target_msg.transform.translation);
  msg::Convert(transform.orientation(), target_msg.transform.rotation);
}

void Base::SetStaticTf2(const cloisim::msgs::Pose transform, const string parent_header_frame_id)
{
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = parent_header_frame_id;
  static_tf.child_frame_id = transform.name();
  msg::Convert(transform.position(), static_tf.transform.translation);
  msg::Convert(transform.orientation(), static_tf.transform.rotation);

  AddStaticTf2(static_tf);
}

cloisim::msgs::Param Base::RequestReplyMessage(
  zmq::Bridge * const bridge_ptr, const string request_message, const string request_value)
{
  cloisim::msgs::Param reply;

  if (bridge_ptr == nullptr) {
    LOG_E(nullptr, "sim bridge is null!!");
    return reply;
  }

  string serialized_request_data;
  cloisim::msgs::Param request;
  if (!request_value.empty()) {
    cloisim::msgs::Any val;
    val.set_type(cloisim::msgs::Any::STRING);
    val.set_string_value(request_value);
    param::Set(request, request_message, val);
  } else {
    param::SetName(request, request_message);
  }

  request.SerializeToString(&serialized_request_data);

  const auto serialized_reply_data = bridge_ptr->RequestReply(serialized_request_data);

  if (serialized_reply_data.size() > 0) {
    if (reply.ParseFromString(serialized_reply_data) == false) {
      LOG_E(nullptr,
        "Failed to parse serialized buffer, buffer_ptr=" << serialized_reply_data.data() <<
          " length=" << serialized_reply_data.size());
    }
  } else {
    LOG_E(nullptr, "Failed to get reply data, length=" << serialized_reply_data.size());
  }

  return reply;
}

cloisim::msgs::Param Base::RequestReplyMessage(
  zmq::Bridge * const bridge_ptr, const cloisim::msgs::Param request_message)
{
  cloisim::msgs::Param response_msg;

  string serializedBuffer;
  request_message.SerializeToString(&serializedBuffer);

  if (bridge_ptr != nullptr) {
    const auto reply_data = bridge_ptr->RequestReply(serializedBuffer);
    response_msg.ParseFromString(reply_data);
  }

  return response_msg;
}

cloisim::msgs::Pose Base::IdentityPose()
{
  cloisim::msgs::Pose identityTransform;
  identityTransform.mutable_position()->set_x(0.0);
  identityTransform.mutable_position()->set_y(0.0);
  identityTransform.mutable_position()->set_z(0.0);
  identityTransform.mutable_orientation()->set_x(0.0);
  identityTransform.mutable_orientation()->set_y(0.0);
  identityTransform.mutable_orientation()->set_z(0.0);
  identityTransform.mutable_orientation()->set_w(1.0);
  return identityTransform;
}

}  // namespace cloisim_ros
