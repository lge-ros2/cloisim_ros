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

#ifndef _CLOISIM_ROS_WORLD_HPP_
#define _CLOISIM_ROS_WORLD_HPP_

#include <cloisim_ros_base/base.hpp>
#include <cloisim_msgs/world_stats.pb.h>
#include <rosgraph_msgs/msg/clock.hpp>

namespace cloisim_ros
{
  class World : public Base
  {
  public:
    explicit World(const rclcpp::NodeOptions &options_, const std::string node_name);
    explicit World();
    virtual ~World();

  private:
    void Initialize() override;
    void Deinitialize() override {};

  private:
    void PublishData(const std::string &buffer);

  private:
    cloisim::msgs::WorldStatistics pb_buf_;

    rosgraph_msgs::msg::Clock msg_clock_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  };
}
#endif