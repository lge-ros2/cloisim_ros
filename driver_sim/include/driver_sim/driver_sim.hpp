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
#include "sim_bridge/sim_bridge.hpp"

class DriverSim : public rclcpp::Node
{
public:
  explicit DriverSim(const std::string node_name);
  ~DriverSim();

protected:
  virtual void Initialize() = 0;
  virtual void Deinitialize() = 0;
  virtual void UpdateData() = 0;

  void Start();
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

  SimBridge* GetSimBridge() { return m_pSimBridge; }

  std::string GetRobotName() { return m_robot_name; }

  void PublishTF();

private:
  void PublishStaticTF();

private:
  SimBridge *m_pSimBridge;

  bool m_bRunThread;
  std::thread m_thread;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Node::SharedPtr m_node_handle;

  std::vector<geometry_msgs::msg::TransformStamped> m_static_tf_list;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

  std::vector<geometry_msgs::msg::TransformStamped> m_tf_list;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  std::string m_robot_name;

protected:
  rclcpp::Time m_simTime;
};
#endif