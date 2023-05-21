/**
 *  @file   sonar.hpp
 *  @date   2023-05-21
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Sonar class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLOISIM_ROS_SONAR_HPP_
#define _CLOISIM_ROS_SONAR_HPP_

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <cloisim_msgs/sonar_stamped.pb.h>

namespace cloisim_ros
{
class Sonar : public Base
{
 public:
  explicit Sonar(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit Sonar(const std::string namespace_ = "");
  ~Sonar();

 private:
  void Initialize() override;
  void Deinitialize() override{};

 private:
  void PublishData(const std::string &buffer);

 private:
  // buffer from simulation
  cloisim::msgs::SonarStamped pb_buf_;

  // Sonar msgs
  sensor_msgs::msg::Range msg_range_;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
};
}  // namespace cloisim_ros
#endif