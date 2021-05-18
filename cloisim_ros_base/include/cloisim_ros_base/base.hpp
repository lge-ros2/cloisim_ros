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

#ifndef _CLOISIM_ROS_BASE_HPP_
#define _CLOISIM_ROS_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cloisim_ros_bridge_zmq/bridge.hpp>
#include <cloisim_msgs/time.pb.h>
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/pose.pb.h>
#include <vector>

namespace cloisim_ros
{
  class Base : public rclcpp::Node
  {
  public:
    explicit Base(const std::string node_name);
    explicit Base(const std::string node_name, const rclcpp::NodeOptions &options);
    explicit Base(const std::string node_name, const std::string namespace_);
    explicit Base(const std::string node_name, const std::string namespace_, const rclcpp::NodeOptions &options);
    ~Base();

  protected:
    virtual void Initialize() = 0;
    virtual void Deinitialize() = 0;
    virtual void UpdatePublishingData(const std::string &buffer) { (void)buffer; };
    virtual void UpdatePublishingData(zmq::Bridge* const bridge_ptr, const std::string &buffer) { (void)bridge_ptr; (void)buffer; };

    void SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const std::string child_frame_id, const std::string header_frame_id = "base_link");
    void SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const cloisim::msgs::Pose transform, const std::string child_frame_id, const std::string header_frame_id = "base_link");
    void SetupStaticTf2(const std::string child_frame_id, const std::string header_frame_id);
    void SetupStaticTf2(const cloisim::msgs::Pose transform, const std::string child_frame_id, const std::string header_frame_id = "base_link");

    void Start();
    void Stop();

    void SetupStaticTf2Message(const cloisim::msgs::Pose transform, const std::string child_frame_id, const std::string parent_frame_id = "base_link");

    void AddTf2(const geometry_msgs::msg::TransformStamped tf)
    {
      m_tf_list.push_back(tf);
    }

    void AddStaticTf2(const geometry_msgs::msg::TransformStamped tf)
    {
      m_static_tf_list.push_back(tf);
    }

    bool IsRunThread() { return m_bRunThread; }

    rclcpp::Node::SharedPtr GetNode() { return m_node_handle; }

    zmq::Bridge* CreateBridge(const std::string hashKey);
    zmq::Bridge* GetBridge(const std::string hashKey);

    void CloseBridges();

    void CreatePublisherThread(zmq::Bridge* const bridge_ptr);

    std::string GetModelName();
    std::string GetRobotName();
    std::string GetPartsName() { return get_name(); }
    std::string GetMainHashKey() { return GetRobotName() + GetPartsName(); }
    std::string GetTargetHashKey(const std::string value) { return GetMainHashKey() + value; }

    void PublishTF();

    cloisim::msgs::Pose GetObjectTransform(zmq::Bridge* const bridge_ptr, const std::string target_name = "");

    void GetRos2Parameter(zmq::Bridge* const bridge_ptr);

    bool GetBufferFromSimulator(zmq::Bridge* const bridge_ptr, void** ppBbuffer, int& bufferLength, const bool isNonBlockingMode = false);

    std::string GetFrameId(const std::string default_frame_id = "base_link");

    void SetSimTime(const cloisim::msgs::Time &time);
    void SetSimTime(const int32_t seconds, const uint32_t nanoseconds);
    rclcpp::Time GetSimTime() { return m_sim_time; }

  public:
    static cloisim::msgs::Param RequestReplyMessage(zmq::Bridge* const bridge_ptr, const std::string request_message, const std::string request_value = "");
    static cloisim::msgs::Param RequestReplyMessage(zmq::Bridge* const bridge_ptr, const cloisim::msgs::Param request_message);
    static cloisim::msgs::Pose IdentityPose();

  private:
    void SetupBridges(const int number_of_bridges);
    void PublishStaticTF();

  private:
    std::map<std::string, zmq::Bridge *> m_hashkey_bridge_map;

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
  };
}
#endif