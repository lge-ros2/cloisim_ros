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
#include <rosgraph_msgs/msg/clock.hpp>

namespace cloisim_ros
{
  class World : public Base
  {
  public:
    explicit World(const rclcpp::NodeOptions &options_, const std::string node_name_);
    explicit World();
    virtual ~World();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdateData(const uint bridge_index) override;

    void PublishSimTime(const rclcpp::Time simTime);

  private:
    std::string hashKeySub_;

    cloisim::msgs::Param pbBuf_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  };
}
#endif