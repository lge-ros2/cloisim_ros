/**
 *  @file   CLidarDriverSim.hpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 lidar Driver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLIDARDRIVERSIM_H_
#define _CLIDARDRIVERSIM_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "sim_bridge/sim_bridge.hpp"
#include "protobuf/laserscan_stamped.pb.h"

class CLidarDriverSim : public rclcpp::Node
{
public:
  CLidarDriverSim();
  virtual ~CLidarDriverSim();
  void SetIntensity(const bool val) { m_bIntensity = val; }
  void SetLowerAngle(const double val) { m_fLowerAngle = val; }
  void SetUpperAngle(const double val) { m_fUpperAngle = val; }

private:
  void DeinitSimConnection();
  void InitSimConnection();
  void Start();
  void Stop();

  void ReadProc();

  void UpdateStaticTF(const rclcpp::Time timestamp);
  void UpdateLaser(const rclcpp::Time timestamp);

private:
  SimBridge *m_pSimBridge;
  bool m_bRun;
  bool m_bIntensity;

  double m_fLowerAngle; // in radians
  double m_fUpperAngle; // in radians

  std::string frame_id_;
  std::vector<double> transform_;
  std::thread m_thread;

  std::string m_hashKeySub;

  gazebo::msgs::LaserScanStamped m_pbBuf;

  rclcpp::Node::SharedPtr node_handle;
  rclcpp::Time m_simTime;

  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // Laser
  sensor_msgs::msg::LaserScan msg_Laser;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaser;

  // Static TF
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
};
#endif