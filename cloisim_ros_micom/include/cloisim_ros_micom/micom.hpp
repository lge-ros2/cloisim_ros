/**
 *  @file   micom.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Micom class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#ifndef CLOISIM_ROS_MICOM__MICOM_HPP_
#define CLOISIM_ROS_MICOM__MICOM_HPP_

#include <cloisim_msgs/micom.pb.h>
#include <cloisim_msgs/param.pb.h>

#include <memory>
#include <string>

#include <cloisim_ros_base/base.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


namespace cloisim_ros
{
class Micom : public Base
{
public:
  explicit Micom(
    const rclcpp::NodeOptions & options_, const std::string node_name,
    const std::string namespace_ = "");
  explicit Micom(const std::string namespace_ = "");
  virtual ~Micom();

private:
  void Initialize() override;
  void Deinitialize() override {}

private:
  void PublishData(const std::string & buffer);

  void ResetOdometryCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/);

  std::string MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  std::string MakeControlMessage(const sensor_msgs::msg::Joy::SharedPtr msg) const;

  std::string MakeMowingBladeHeightMessage(const std_msgs::msg::Float32::SharedPtr msg) const;

  std::string MakeMowingRevSpeedMessage(const std_msgs::msg::UInt16::SharedPtr msg) const;

  void UpdateOdom();
  void UpdateImu();
  void UpdateBattery();
  void UpdateBumper();
  void UpdateIR();
  void UpdateUSS();

private:
  zmq::Bridge * info_bridge_ptr;

  // Micom msgs
  cloisim::msgs::Micom pb_micom_;

  // IMU msgs
  sensor_msgs::msg::Imu msg_imu_;

  geometry_msgs::msg::TransformStamped odom_tf_;

  // Odometry msgs
  nav_msgs::msg::Odometry msg_odom_;

  // Battery
  sensor_msgs::msg::BatteryState msg_battery_;

  // Bumper
  std_msgs::msg::UInt8MultiArray msg_bumper_;

  // USS
  std_msgs::msg::Float64MultiArray msg_uss_;
  geometry_msgs::msg::PoseArray msg_uss_posearray_;

  // IR
  std_msgs::msg::Float64MultiArray msg_ir_;
  geometry_msgs::msg::PoseArray msg_ir_posearray_;

  // ROS2 micom publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_bumper_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ir_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_ir_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_uss_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_uss_pose_;

  // wheel command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

  // joy command subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  // set height of blade
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_blade_height_;

  // set rev speed of blade
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_blade_rev_speed_;

  // reset odometry pose service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_odom_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_MICOM__MICOM_HPP_
