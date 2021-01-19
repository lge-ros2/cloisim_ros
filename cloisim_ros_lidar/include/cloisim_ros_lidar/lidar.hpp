/**
 *  @file   LidarDriverSim.hpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 lidar class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLOISIM_ROS_LIDAR_HPP_
#define _CLOISIM_ROS_LIDAR_HPP_

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cloisim_msgs/laserscan_stamped.pb.h>

namespace cloisim_ros
{
  class Lidar : public Base
  {
  public:
    explicit Lidar(const rclcpp::NodeOptions &options_, const std::string node_name_, const std::string namespace_ = "");
    explicit Lidar(const std::string namespace_ = "");
    ~Lidar();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdateData(const uint bridge_index) override;

  private:
    void UpdateLaserData();

  private:
    // key for connection
    std::string hashKeySub_;

    // buffer from simulation
    cloisim::msgs::LaserScanStamped pbBuf_;

    // message for ROS2 communictaion
    sensor_msgs::msg::LaserScan msg_Laser;

    // Laser publisher
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaser;
  };
}
#endif