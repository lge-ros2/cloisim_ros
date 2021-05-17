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
    explicit Micom(const rclcpp::NodeOptions &options_, const std::string node_name_, const std::string namespace_ = "");
    explicit Micom(const std::string namespace_ = "");
    virtual ~Micom();

  private:
    void Initialize() override;
    void Deinitialize() override { };
    void UpdatePublishingData(const std::string &buffer) override;

  private:
    void GetWeelInfo(zmq::Bridge *const pBridge);
    void GetTransformNameInfo(zmq::Bridge *const pBridge);

    void MicomWrite(zmq::Bridge* const pBridge, const std::string &buffer);

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
    cloisim_ros::zmq::Bridge *pBridgeInfo;

    std::map<std::string, std::string> target_transform_name;

    double wheel_tread;
    double wheel_radius;
    std::string base_link_name_;

    // Micom msgs
    cloisim::msgs::Micom pbMicom;

    std::array<double, 2> last_rad_ = {0, 0};

    // IMU msgs
    sensor_msgs::msg::Imu msgImu;

    // Odometry msgs
    geometry_msgs::msg::TransformStamped odom_tf_;
    geometry_msgs::msg::TransformStamped wheel_left_tf_;
    geometry_msgs::msg::TransformStamped wheel_right_tf_;
    nav_msgs::msg::Odometry msgOdom;

    std::array<double, 3> orig_left_wheel_rot_;
    std::array<double, 3> orig_right_wheel_rot_;

    // Battery
    sensor_msgs::msg::BatteryState msg_battery_;

    // ROS2 micom publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pubBatteryState_;

    // wheel command subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMicom_;

    // reset odometry pose service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvResetOdom_;
  };
}
#endif
