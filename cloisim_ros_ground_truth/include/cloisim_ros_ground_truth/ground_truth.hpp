/**
 *  @file   ground_truth.hpp
 *  @date   2021-05-16
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

#ifndef _cloisim_ros_ground_truth_HPP_
#define _cloisim_ros_ground_truth_HPP_

#include <cloisim_ros_base/base.hpp>
#include <cloisim_msgs/perception_v.pb.h>
#include <perception_msgs/msg/object_array.hpp>

namespace cloisim_ros
{
  class GroundTruth : public Base
  {
  public:
    explicit GroundTruth(const rclcpp::NodeOptions &options_, const std::string node_name);
    explicit GroundTruth();
    virtual ~GroundTruth();

  private:
    void Initialize() override;
    void Deinitialize() override { };
    void UpdatePublishingData(const std::string &buffer) override;

    void UpdatePerceptionData();

  private:
    cloisim::msgs::Perception_V pb_buf_;

    perception_msgs::msg::ObjectArray msg_;

    rclcpp::Publisher<perception_msgs::msg::ObjectArray>::SharedPtr pub_;
  };
}
#endif