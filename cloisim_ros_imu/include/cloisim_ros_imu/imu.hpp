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

#ifndef _CLOISIM_ROS_IMU_HPP_
#define _CLOISIM_ROS_IMU_HPP_

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cloisim_msgs/imu.pb.h>

namespace cloisim_ros
{
  class Imu : public Base
  {
  public:
    explicit Imu(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
    explicit Imu(const std::string namespace_ = "");
    ~Imu();

  private:
    void Initialize() override;
    void Deinitialize() override { };

  private:
    void PublishData(const std::string &buffer);

  private:
    // buffer from simulation
    cloisim::msgs::IMU pb_buf_;

    // IMU msgs
    sensor_msgs::msg::Imu msg_imu_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  };
}
#endif