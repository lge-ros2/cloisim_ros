/**
 *  @file   unity_ros.hpp
 *  @date   2020-04-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 node for controlling unity simulation
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _UnityRos_HPP_
#define _UnityRos_HPP_

#include <sim_bridge/sim_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace UnityRos
{
class Throttler;

class UnityRosInit : public rclcpp::Node
{
public:
  UnityRosInit();
  virtual ~UnityRosInit();

private:
  void PublishSimTime(const rclcpp::Time simTime, const rclcpp::Time realTime);

  void Start();
  void Stop();
  void RxProc();

private:
  SimBridge *m_pSimBridge;
  uint16_t portClock_;
  std::string m_hashKey;
  Throttler *throttler_;
  std::thread m_thread;
  bool m_bRun;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
};

class Throttler
{
private:
  double period_;
  rclcpp::Time last_time_;

public:
  Throttler(const double _hz)
  : period_(1.0 / _hz),
    last_time_(0)
  {
  }

  bool IsReady(const rclcpp::Time & _now)
  {
    // If time went backwards, reset
    if (_now < last_time_)
    {
      // printf("backward %.10f, %.10f\n", last_time_.seconds(), _now.seconds());
      last_time_ = _now;
      return true;
    }

    // If not enough time has passed, return false
    double elapsedTime = (_now - last_time_).seconds();

    if (period_ > 0 && elapsedTime < period_)
    {
      // printf("FALSE %.10f, %.10f\n", elapsedTime, _now.seconds());
      return false;
    }

    // Enough time has passed, set last time to now and return true
    last_time_ = _now;
    // printf("TRUE %.10f , %.10f\n", last_time_.seconds(),  _now.seconds());
    return true;
  }
};
}
#endif