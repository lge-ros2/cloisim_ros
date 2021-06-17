/**
 *  @file   micom.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Micom class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#ifndef _CLOISIM_ROS_MICOM_HPP__
#define _CLOISIM_ROS_MICOM_HPP__

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cloisim_msgs/micom.pb.h>

namespace cloisim_ros
{
  class Micom : public Base
  {
  public:
    explicit Micom(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
    explicit Micom(const std::string namespace_ = "");
    virtual ~Micom();

  private:
    void Initialize() override;
    void Deinitialize() override { };

  private:
    void PublishData(const std::string &buffer);
    void GetTransformNameInfo(zmq::Bridge *const bridge_ptr);

    void ResetOdometryCallback(
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/);

    std::string MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const;

    bool CalculateOdometry(const rclcpp::Duration duration, const double _wheel_angular_vel_left, const double _wheel_angular_vel_right, const double _theta);

    void UpdateOdom();
    void UpdateImu();
    void UpdateBattery();

  private:
    zmq::Bridge *info_bridge_ptr;

    std::map<std::string, std::string> target_transform_name;

    // Micom msgs
    cloisim::msgs::Micom pb_micom_;

    std::array<double, 2> last_rad_ = {0, 0};

    // IMU msgs
    sensor_msgs::msg::Imu msg_imu_;

    // Odometry msgs
    geometry_msgs::msg::TransformStamped odom_tf_;
    geometry_msgs::msg::TransformStamped wheel_left_tf_;
    geometry_msgs::msg::TransformStamped wheel_right_tf_;
    nav_msgs::msg::Odometry msg_odom_;

    std::array<double, 3> orig_left_wheel_rot_;
    std::array<double, 3> orig_right_wheel_rot_;

    // Battery
    sensor_msgs::msg::BatteryState msg_battery_;

    // ROS2 micom publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;

    // wheel command subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_micom_;

    // reset odometry pose service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_odom_;
  };
}
#endif
