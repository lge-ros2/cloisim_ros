/**
 *  @file   range.hpp
 *  @date   2025-02-05
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Range class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_RANGE__RANGE_HPP_
#define CLOISIM_ROS_RANGE__RANGE_HPP_

#include <cloisim_msgs/sonar_stamped.pb.h>

#include <string>

#include <cloisim_ros_base/base.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace cloisim_ros
{
class Range : public Base
{
public:
  explicit Range(
    const rclcpp::NodeOptions & options_, const std::string node_name,
    const std::string namespace_ = "");
  explicit Range(const std::string namespace_ = "");
  ~Range();

  void SetRadiationType(uint8_t value) {radiation_type_ = value;}

private:
  void Initialize() override;
  void Deinitialize() override {}

private:
  void PublishData(const std::string & buffer);

private:
  uint8_t radiation_type_;

  // buffer from simulation
  cloisim::msgs::SonarStamped pb_buf_;

  sensor_msgs::msg::Range msg_range_;
  geometry_msgs::msg::PoseStamped msg_pose_;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_RANGE__RANGE_HPP_
