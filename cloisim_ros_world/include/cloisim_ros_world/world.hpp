/**
 *  @file   cloisim_ros_world.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 node for controlling unity simulation
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#pragma once
#ifndef CLOISIM_ROS_WORLD__WORLD_HPP_
#define CLOISIM_ROS_WORLD__WORLD_HPP_

#include <cloisim_msgs/world_stats.pb.h>

#include <string>

#include <cloisim_ros_base/base.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>

namespace cloisim_ros
{
class World : public Base
{
public:
  explicit World(const rclcpp::NodeOptions & options_, const std::string node_name);
  World();
  ~World();

private:
  void Initialize() override;
  void Deinitialize() override;

private:
  void PublishData(const std::string & buffer);
  std::string ServiceRequest(const std::string & buffer);

  std::vector<std::string> FindResetTimeServices() const;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr GetOrCreateResetClient(
    const std::string & service_name);

private:
  cloisim::msgs::WorldStatistics pb_buf_;

  rosgraph_msgs::msg::Clock msg_clock_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;

  mutable std::mutex rviz_client_mtx_;
  std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Empty>::SharedPtr> rviz_clients_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_WORLD__WORLD_HPP_
