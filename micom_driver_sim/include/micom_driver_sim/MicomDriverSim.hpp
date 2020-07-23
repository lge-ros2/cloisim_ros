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

#include "driver_sim/driver_sim.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "protobuf/micom.pb.h"

class MicomDriverSim : public DriverSim
{
public:
  static constexpr double WHEEL_RADIUS_RATIO = 1.00f;
  static constexpr double RAD2DEG = 57.29577951f; ///< 180.0/PI
  static constexpr double DEG2RAD = 0.017453292f; ///< PI/180.0

public:
  MicomDriverSim();
  virtual ~MicomDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const int bridge_index) override;

private:
  void MicomWrite(const void* const pcBuf, const uint32_t unSize);

  std::string MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  bool CalculateOdometry(const rclcpp::Duration duration,
      const double _wheel_left, const double _wheel_right, const double _theta);

  void UpdateOdom();
  void UpdateImu();
  void UpdateBattery();

private:
  std::string m_hashKeyPub;
  std::string m_hashKeySub;

  // Yaml parameters
  double wheel_base;
  double wheel_radius;

  bool m_use_pub;
  bool m_use_sub;

  // Micom msgs
  gazebo::msgs::Micom m_pbBufMicom;

  std::array<double, 3> odom_pose;
  std::array<double, 3> odom_vel;
  std::array<double, 2> last_rad;

  // IMU msgs
  sensor_msgs::msg::Imu msg_imu;

  // Odometry msgs
  geometry_msgs::msg::TransformStamped odom_tf;
  geometry_msgs::msg::TransformStamped wheel_left_tf;
  geometry_msgs::msg::TransformStamped wheel_right_tf;
  nav_msgs::msg::Odometry msg_odom;

  // Battery
  sensor_msgs::msg::BatteryState msg_battery;

  // ROS2 micom publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pubBatteryState;

  // wheel command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMicom;
};
#endif
