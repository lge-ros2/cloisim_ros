/**
 *  @file   LidarDriverSim.hpp
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

#ifndef _LIDARDRIVERSIM_H_
#define _LIDARDRIVERSIM_H_

#include <driver_sim/driver_sim.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <protobuf/laserscan_stamped.pb.h>

class LidarDriverSim : public DriverSim
{
public:
  LidarDriverSim();
  ~LidarDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index) override;

private:
  void UpdateLaserData();

private:
  // key for connection
  uint16_t portData_;
  std::string m_hashKeySub;

  // buffer from simulation
  gazebo::msgs::LaserScanStamped m_pbBuf;

  // message for ROS2 communictaion
  sensor_msgs::msg::LaserScan msg_Laser;

  // Laser publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaser;
};
#endif