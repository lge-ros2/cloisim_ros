/**
 *  @file   driver_sim.hpp
 *  @date   2020-05-22
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 driver Sim base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _DRIVER_SIM_HPP_
#define _DRIVER_SIM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sim_bridge/sim_bridge.hpp>
#include <protobuf/param.pb.h>
#include <protobuf/pose.pb.h>
#include <vector>

class DriverSim : public rclcpp::Node
{
public:
  explicit DriverSim(const std::string node_name, const int number_of_simbridge = 1);
  ~DriverSim();

protected:
  virtual void Initialize() = 0;
  virtual void Deinitialize() = 0;
  virtual void UpdateData(const uint bridge_index = 0) = 0; // Function called at loop thread
  virtual void InitializeTfMessage(const gazebo::msgs::Pose transform, const std::string frame_id) = 0;

  void Start(const bool runDefaultUpdateDataThread = true);
  void Stop();

  void AddTf2(const geometry_msgs::msg::TransformStamped _tf)
  {
    m_tf_list.push_back(_tf);
  }

  void AddStaticTf2(const geometry_msgs::msg::TransformStamped _tf)
  {
    m_static_tf_list.push_back(_tf);
  }

  bool IsRunThread() { return m_bRunThread; }

  rclcpp::Node* GetNode() { return m_node_handle.get(); }

  SimBridge* GetSimBridge(const uint bridge_index = 0);

  void DisconnectSimBridges();

  std::string GetRobotName() { return m_robot_name; }
  std::string GetPartsName() { return m_parts_name; }

  void PublishTF();

  gazebo::msgs::Pose GetObjectTransform(SimBridge* const simBridge, const std::string target_name = "");
  gazebo::msgs::Pose GetObjectTransform(const int bridge_index, const std::string target_name = "");

private:
  void PublishStaticTF();

private:
  std::vector<SimBridge *> m_simBridgeList;

  bool m_bRunThread;
  std::thread m_thread;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Node::SharedPtr m_node_handle;

  std::vector<geometry_msgs::msg::TransformStamped> m_static_tf_list;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

  std::vector<geometry_msgs::msg::TransformStamped> m_tf_list;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  std::string m_robot_name;
  std::string m_parts_name;

protected:
  rclcpp::Time m_simTime;
};
#endif