/**
 *  @file   MicomDriverSim.hpp
 *  @date   2020-05-25Z
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 MicomDriver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#ifndef _MICOMDRIVERSIM_HPP__
#define _MICOMDRIVERSIM_HPP__

#include <driver_sim/driver_sim.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <protobuf/micom.pb.h>

class MicomDriverSim : public DriverSim
{
public:
  MicomDriverSim();
  virtual ~MicomDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index = 0) override;

private:
  void GetWeelInfo(SimBridge* const pSimBridge);
  void GetTransformNameInfo(SimBridge* const pSimBridge);

  void MicomWrite(const void* const pcBuf, const uint32_t unSize);

  void ResetOdometryCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/);

  std::string MakeCommandMessage(const std::string command);
  std::string MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  bool CalculateOdometry(const rclcpp::Duration duration, const double _wheel_angular_vel_left, const double _wheel_angular_vel_right, const double _theta);

  void UpdateOdom();
  void UpdateImu();
  void UpdateBattery();

private:
  uint16_t portTx_;
  uint16_t portRx_;
  std::string m_hashKeyPub;
  std::string m_hashKeySub;

  std::map<std::string, std::string> target_transform_name;

  double wheel_base;
  double wheel_radius;
  std::string base_link_name_;

  bool use_pub_;
  bool use_sub_;

  // Micom msgs
  gazebo::msgs::Micom pbBufMicom_;

  std::array<double, 2> last_rad_;

  // IMU msgs
  sensor_msgs::msg::Imu msg_imu_;

  // Odometry msgs
  geometry_msgs::msg::TransformStamped odom_tf_;
  geometry_msgs::msg::TransformStamped wheel_left_tf_;
  geometry_msgs::msg::TransformStamped wheel_right_tf_;
  nav_msgs::msg::Odometry msg_odom_;

  std::array<double, 3> orig_left_wheel_rot_;
  std::array<double, 3> orig_right_wheel_rot_;

  // Battery
  sensor_msgs::msg::BatteryState msg_battery_;

  // ROS2 micom publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pubBatteryState_;

  // wheel command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMicom_;

  // reset odometry pose service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvResetOdom_;
};
#endif
