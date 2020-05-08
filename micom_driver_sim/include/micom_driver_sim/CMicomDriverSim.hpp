/**
 *  @file   CMicomDriversim.hpp
 *  @date   2019-03-28
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
#ifndef _CMICOMDRIVERSIM_HPP__
#define _CMICOMDRIVERSIM_HPP__

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include "sim_bridge/sim_bridge.hpp"
#include "protobuf/micom.pb.h"

class CMicomDriverSim : public rclcpp::Node
{
public:
  CMicomDriverSim();
  virtual ~CMicomDriverSim();

private:
  void Start();
  void Stop();

  void MicomProc();
  bool MicomWrite(const void* const pcBuf, const uint32_t unSize);

  std::string MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  bool CalculateOdometry(const rclcpp::Duration duration,
      const double _wheel_left, const double _wheel_right, const double _theta);
  void UpdateOdom(const rclcpp::Time timestamp);
  void UpdateImu(const rclcpp::Time timestamp);
  void UpdateBattery(const rclcpp::Time timestamp);
  void UpdateTF(const rclcpp::Time timestamp);
  void UpdateStaticTF(const rclcpp::Time timestamp);

public:
  static constexpr double WHEEL_RADIUS_RATIO = 1.00f;
  static constexpr double RAD2DEG = 57.29577951f; ///< 180.0/PI
  static constexpr double DEG2RAD = 0.017453292f; ///< PI/180.0
  static constexpr double DEF_MAX_DEL_THETA_DEGREE = 1.5;

private:
  SimBridge *m_pSimBridge;
  static const uint32_t ZMQ_BUFSIZE = 30 * 1024;

  bool m_bMicomThreadFlag;
  std::thread m_thread;

  double m_fThreadMicomReadPeriodMs;

  std::string m_hashKeyPub;
  std::string m_hashKeySub;

  rclcpp::Time m_SimTime;

  // Micom msgs
  gazebo::msgs::Micom m_MicomData;

  std::array<double, 3> odom_pose;
  std::array<double, 3> odom_vel;
  std::array<double, 2> last_rad;

  double wheel_base;
  double wheel_radius;

  bool m_use_pub;
  bool m_use_sub;

  rclcpp::Node::SharedPtr node_handle;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_tf_static;

  // IMU
  sensor_msgs::msg::Imu msg_imu;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImuMsg;

  // Odometry
  nav_msgs::msg::Odometry msg_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometryMsg;

  // Battery
  sensor_msgs::msg::BatteryState msg_battery;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pubBatteryStateMsg;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Static TF
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  // wheel command
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMobileMCUMsg;
};
#endif
