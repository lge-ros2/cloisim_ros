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
    explicit Gps(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
    explicit Gps(const std::string namespace_ = "");
    ~Gps();

  private:
    void Initialize() override;
    void Deinitialize() override { };

  private:
    void PublishData(const std::string &buffer);

  private:
    // buffer from simulation
    cloisim::msgs::GPS pb_buf_;

    // message for ROS2 communictaion
    sensor_msgs::msg::NavSatFix msg_nav_;

    // Laser publisher
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  };
}
#endif