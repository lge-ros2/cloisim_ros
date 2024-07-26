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

#ifndef CLOISIM_ROS_GROUND_TRUTH__GROUND_TRUTH_HPP_
#define CLOISIM_ROS_GROUND_TRUTH__GROUND_TRUTH_HPP_

#include <cloisim_msgs/perception_v.pb.h>

#include <string>

#include <cloisim_ros_base/base.hpp>
#include <perception_msgs/msg/object_array.hpp>

namespace cloisim_ros
{
class GroundTruth : public Base
{
public:
  explicit GroundTruth(const rclcpp::NodeOptions & options_, const std::string node_name);
  GroundTruth();
  virtual ~GroundTruth();

private:
  void Initialize() override;
  void Deinitialize() override {}

  void PublishData(const std::string & buffer);
  void UpdatePerceptionData();

private:
  cloisim::msgs::Perception_V pb_buf_;

  perception_msgs::msg::ObjectArray msg_;

  rclcpp::Publisher<perception_msgs::msg::ObjectArray>::SharedPtr pub_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_GROUND_TRUTH__GROUND_TRUTH_HPP_
