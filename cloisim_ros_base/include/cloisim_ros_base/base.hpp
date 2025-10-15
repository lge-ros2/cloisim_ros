/**
 *  @file   cloisim_ros_base.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CLOiSim-ROS base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_BASE__BASE_HPP_
#define CLOISIM_ROS_BASE__BASE_HPP_

#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/pose.pb.h>
#include <cloisim_msgs/time.pb.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

#include <cloisim_ros_base/helper.hpp>
#include <cloisim_ros_bridge_zmq/bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#define INFO_ONCE RCLCPP_INFO_STREAM_ONCE
#define WARN_ONCE RCLCPP_WARN_STREAM_ONCE
#define ERR_ONCE RCLCPP_ERROR_STREAM_ONCE
#define INFO RCLCPP_INFO_STREAM
#define WARN RCLCPP_WARN_STREAM
#define ERR RCLCPP_ERROR_STREAM

namespace cloisim_ros
{
class Base : public rclcpp::Node
{
public:
  explicit Base(const std::string node_name);
  explicit Base(const std::string node_name, const rclcpp::NodeOptions & options);
  explicit Base(const std::string node_name, const std::string namespace_);
  explicit Base(
    const std::string node_name, const std::string namespace_, const rclcpp::NodeOptions & options);
  ~Base();

protected:
  virtual void Initialize() = 0;
  virtual void Deinitialize() = 0;

protected:
  void SetTf2(
    geometry_msgs::msg::TransformStamped & target_msg, const std::string child_frame_id,
    const std::string header_frame_id = "base_link");

  void SetTf2(
    geometry_msgs::msg::TransformStamped & target_msg, const cloisim::msgs::Pose transform,
    const std::string child_frame_id, const std::string header_frame_id = "base_link");

  void SetStaticTf2(
    const cloisim::msgs::Pose transform, const std::string parent_header_frame_id = "base_link");

  void Start();
  void Start(const bool enable_tf_publish);
  void Stop();

  void AddTf2(const geometry_msgs::msg::TransformStamped tf);
  void AddStaticTf2(const geometry_msgs::msg::TransformStamped tf);

  bool IsRunThread() {return m_bRunThread;}

  rclcpp::Node::SharedPtr GetNode() {return m_node_handle;}

  zmq::Bridge * CreateBridge();

  void CloseBridges();

  void AddBridgeReceiveWorker(
    zmq::Bridge * const bridge_ptr, std::function<void(const std::string &)> thread_func);

  std::string GetModelName();
  std::string GetRobotName();
  std::string GetPartsName() {return get_name();}
  std::string GetMainHashKey() {return GetRobotName() + GetPartsName();}
  std::string GetTargetHashKey(const std::string value) {return GetMainHashKey() + value;}

  void PublishTF();
  void PublishTF(const geometry_msgs::msg::TransformStamped & tf);

  void SetStaticTransforms(zmq::Bridge * const bridge_ptr);

  cloisim::msgs::Pose GetObjectTransform(
    zmq::Bridge * const bridge_ptr, const std::string target_name = "");

  cloisim::msgs::Pose GetObjectTransform(
    zmq::Bridge * const bridge_ptr, const std::string target_name, std::string & parent_frame_id);

  void GetRos2Parameter(zmq::Bridge * const bridge_ptr);

  static bool GetBufferFromSimulator(
    zmq::Bridge * const bridge_ptr, void ** ppBbuffer, int & bufferLength,
    const bool isNonBlockingMode = false);

  static bool SetBufferToSimulator(zmq::Bridge * const bridge_ptr, const std::string & buffer);

  std::string GetFrameId(const std::string default_frame_id = "base_link");

  static cloisim::msgs::Param RequestReplyMessage(
    zmq::Bridge * const bridge_ptr, const std::string request_message,
    const std::string request_value = "");
  static cloisim::msgs::Param RequestReplyMessage(
    zmq::Bridge * const bridge_ptr, const cloisim::msgs::Param request_message);
  static cloisim::msgs::Pose IdentityPose();

  void SetTime(const cloisim::msgs::Time & time);
  void SetTime(const int32_t seconds, const uint32_t nanoseconds);
  rclcpp::Time GetTime() {return m_sim_time;}

public:
  void GenerateTF(const std::string & buffer);

private:
  void PublishStaticTF();

private:
  std::vector<std::unique_ptr<zmq::Bridge>> m_created_bridges;

  bool m_bRunThread;
  std::vector<std::thread> m_threads;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Node::SharedPtr m_node_handle;

  rclcpp::Time m_sim_time;

  std::vector<geometry_msgs::msg::TransformStamped> m_static_tf_list;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

  std::vector<geometry_msgs::msg::TransformStamped> m_tf_list;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

protected:
  // for ros2 default parameters
  std::string topic_name_;
  std::vector<std::string> frame_id_list_;

  bool enable_tf_publish_;
};

inline void Base::Start() {Start(enable_tf_publish_);}

inline void Base::AddTf2(const geometry_msgs::msg::TransformStamped tf) {m_tf_list.push_back(tf);}

inline void Base::AddStaticTf2(const geometry_msgs::msg::TransformStamped tf)
{
  m_static_tf_list.push_back(tf);
}

inline void Base::PublishTF(const geometry_msgs::msg::TransformStamped & tf)
{
  if (m_tf_broadcaster != nullptr) {
    m_tf_broadcaster->sendTransform(tf);
  }
}

inline cloisim::msgs::Pose Base::GetObjectTransform(
  zmq::Bridge * const bridge_ptr, const std::string target_name)
{
  std::string empty_arg("");
  return GetObjectTransform(bridge_ptr, target_name, empty_arg);
}

inline bool Base::SetBufferToSimulator(zmq::Bridge * const bridge_ptr, const std::string & buffer)
{
  if (!buffer.empty() && buffer.size() > 0 && bridge_ptr != nullptr) {
    return bridge_ptr->Send(buffer.data(), buffer.size());
  } else {
    return false;
  }
}

inline void Base::SetTf2(
  geometry_msgs::msg::TransformStamped & target_msg, const std::string child_frame_id,
  const std::string header_frame_id)
{
  SetTf2(target_msg, IdentityPose(), child_frame_id, header_frame_id);
}

inline std::string Base::GetFrameId(const std::string default_frame_id)
{
  return (frame_id_list_.size() == 0) ? default_frame_id : frame_id_list_.back();
}

inline void Base::SetTime(const cloisim::msgs::Time & time) {SetTime(time.sec(), time.nsec());}

inline void Base::SetTime(const int32_t seconds, const uint32_t nanoseconds)
{
  m_sim_time = rclcpp::Time(seconds, nanoseconds);
}
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_BASE__BASE_HPP_
