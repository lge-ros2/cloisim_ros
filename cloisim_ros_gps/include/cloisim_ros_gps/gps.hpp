/**
 *  @file   gps.hpp
 *  @date   2021-01-14
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

#ifndef _CLOISIM_ROS_GPS_HPP_
#define _CLOISIM_ROS_GPS_HPP_

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cloisim_msgs/gps.pb.h>

namespace cloisim_ros
{
  class Gps : public Base
  {
  public:
    explicit Gps(const std::string node_name = "cloisim_ros_gps");
    ~Gps();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdateData(const uint bridge_index) override;

  private:
    // key for connection
    uint16_t portData_;
    std::string hashKeySub_;

    // buffer from simulation
    cloisim::msgs::GPS pbBuf_;

    // message for ROS2 communictaion
    sensor_msgs::msg::NavSatFix msg_navsat;

    // Laser publisher
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubNav;
  };
} // namespace cloisim_ros
#endif