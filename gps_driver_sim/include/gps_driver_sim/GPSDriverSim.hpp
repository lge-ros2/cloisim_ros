/**
 *  @file   GPSDriverSim.hpp
 *  @date   2020-06-26
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 GPS Driver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _GPSDRIVERSIM_H_
#define _GPSDRIVERSIM_H_

#include <driver_sim/driver_sim.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <protobuf/gps.pb.h>

class GPSDriverSim : public DriverSim
{
public:
  GPSDriverSim();
  ~GPSDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index) override;
  virtual void InitializeTfMessage(const gazebo::msgs::Pose transform, const std::string frame_id) override;

private:
  // key for connection
  std::string m_hashKeySub;

  // buffer from simulation
  gazebo::msgs::GPS m_pbBuf;

  // message for ROS2 communictaion
  sensor_msgs::msg::NavSatFix msg_navsat;

  // Laser publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubNav;
};
#endif