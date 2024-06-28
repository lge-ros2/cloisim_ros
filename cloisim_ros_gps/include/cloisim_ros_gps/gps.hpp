/**
 *  @file   gps.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 GPS class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_GPS__GPS_HPP_
#define CLOISIM_ROS_GPS__GPS_HPP_

#include <cloisim_msgs/gps.pb.h>

#include <string>

#include <cloisim_ros_base/base.hpp>
#include <cloisim_ros_base/helper.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace cloisim_ros
{
class Gps : public Base
{
 public:
  explicit Gps(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit Gps(const std::string namespace_ = "");
  ~Gps();

 private:
  void Initialize() override;
  void Deinitialize() override{};

 private:
  void PublishData(const std::string &buffer);

 private:
  std::string topic_name_heading_;

  // buffer from simulation
  cloisim::msgs::GPS pb_buf_gps_;
  cloisim::msgs::IMU pb_buf_imu_;

  // message for ROS2 communictaion
  sensor_msgs::msg::NavSatFix msg_gps_;
  sensor_msgs::msg::Imu msg_heading_;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_heading_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_GPS__GPS_HPP_
