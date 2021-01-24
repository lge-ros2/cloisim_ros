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
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/pose.pb.h>
#include <vector>

namespace cloisim_ros
{
  class Base : public rclcpp::Node
  {
  public:
    explicit Base(const std::string node_name_, const int number_of_bridges = 1);
    explicit Base(const std::string node_name_, const rclcpp::NodeOptions &options_, const int number_of_bridges = 1);
    explicit Base(const std::string node_name_, const std::string namespace_, const int number_of_bridges = 1);
    explicit Base(const std::string node_name_, const std::string namespace_, const rclcpp::NodeOptions &options_, const int number_of_bridges = 1);
    ~Base();

  protected:
    virtual void Initialize() = 0;
    virtual void Deinitialize() = 0;
    virtual void UpdateData(const uint bridge_index = 0) = 0; // Function called at loop thread

    void SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const std::string child_frame_id, const std::string header_frame_id = "base_link");
    void SetTf2(geometry_msgs::msg::TransformStamped& target_msg, const cloisim::msgs::Pose transform, const std::string child_frame_id, const std::string header_frame_id = "base_link");
    void SetupStaticTf2(const std::string child_frame_id, const std::string header_frame_id);
    void SetupStaticTf2(const cloisim::msgs::Pose transform, const std::string child_frame_id, const std::string header_frame_id = "base_link");

    void Start(const bool runSingleDataThread = true);
    void Stop();

    void SetupStaticTf2Message(const cloisim::msgs::Pose transform, const std::string child_frame_id, const std::string parent_frame_id = "base_link");

    void AddTf2(const geometry_msgs::msg::TransformStamped _tf)
    {
      m_tf_list.push_back(_tf);
    }

    void AddStaticTf2(const geometry_msgs::msg::TransformStamped _tf)
    {
      m_static_tf_list.push_back(_tf);
    }

    bool IsRunThread() { return m_bRunThread; }

    rclcpp::Node::SharedPtr GetNode() { return m_node_handle; }

    zmq::Bridge* GetBridge(const uint bridge_index = 0);

    void CloseBridges();

    std::string GetRobotName();
    std::string GetPartsName() { return get_name(); }
    std::string GetMainHashKey() { return GetRobotName() + GetPartsName(); }

    void PublishTF();

    cloisim::msgs::Pose GetObjectTransform(zmq::Bridge* const pBridge, const std::string target_name = "");
    cloisim::msgs::Pose GetObjectTransform(const int bridge_index, const std::string target_name = "");

    void GetRos2Parameter(zmq::Bridge* const pBridge);

    bool GetBufferFromSimulator(const uint bridge_index, void** ppBbuffer, int& bufferLength, const bool isNonBlockingMode = false);

  public:
    static cloisim::msgs::Param RequestReplyMessage(zmq::Bridge* const pBridge, const cloisim::msgs::Param request_message);
    static cloisim::msgs::Pose IdentityPose();

  private:
    void SetupBridges(const int number_of_bridges);
    void PublishStaticTF();

  private:
    std::vector<zmq::Bridge *> m_bridgeList;

    bool m_bRunThread;
    std::thread m_thread;

    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Node::SharedPtr m_node_handle;

    std::vector<geometry_msgs::msg::TransformStamped> m_static_tf_list;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

    std::vector<geometry_msgs::msg::TransformStamped> m_tf_list;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  protected:
    rclcpp::Time m_simTime;

    std::string topic_name_;
    std::string frame_id_;
  };
}
#endif