/**
 *  @file   micom.cpp
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

#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/twist.pb.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_micom/micom.hpp"
#include <cloisim_ros_base/helper.hpp>

using namespace std::literals::chrono_literals;
using namespace std::placeholders;
using string = std::string;

namespace cloisim_ros
{

Micom::Micom(const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: Base(node_name, namespace_, options_)
{
  Start();
}

Micom::Micom(const string namespace_)
: Micom(rclcpp::NodeOptions(), "cloisim_ros_micom", namespace_)
{
}

Micom::~Micom() {Stop();}

void Micom::Initialize()
{
  uint16_t portInfo, portTx, portRx, portTf;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));
  get_parameter_or("bridge.Tx", portTx, uint16_t(0));
  get_parameter_or("bridge.Rx", portRx, uint16_t(0));
  get_parameter_or("bridge.Tf", portTf, uint16_t(0));
  const auto hashKeyInfo = GetTargetHashKey("Info");
  const auto hashKeyPub = GetTargetHashKey("Rx");
  const auto hashKeySub = GetTargetHashKey("Tx");
  const auto hashKeyTf = GetTargetHashKey("Tf");
  DBG_SIM_INFO(
    "hashKey: info(%s) pub(%s) sub(%s) tf(%s)", hashKeyInfo.c_str(), hashKeyPub.c_str(),
    hashKeySub.c_str(), hashKeyTf.c_str());

  {
    auto base_link_pose = IdentityPose();
    base_link_pose.set_name("base_link");
    SetStaticTf2(base_link_pose, "base_footprint");
  }

  info_bridge_ptr = CreateBridge();
  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);
    GetStaticTransforms(info_bridge_ptr);
  }

  pub_battery_ =
    create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SensorDataQoS());

  {
    msg_odom_.header.frame_id = "odom";
    msg_odom_.child_frame_id = "base_footprint";

    SetTf2(odom_tf_, msg_odom_.child_frame_id, msg_odom_.header.frame_id);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
  }

  {
    msg_imu_.header.frame_id = "imu_link";

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
  }

  auto data_bridge_ptr = CreateBridge();
  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::PUB, portRx, hashKeyPub);
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portTx, hashKeySub);
    AddPublisherThread(data_bridge_ptr, bind(&Micom::PublishData, this, std::placeholders::_1));
  }

  auto tf_bridge_ptr = CreateBridge();
  if (tf_bridge_ptr != nullptr) {
    tf_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portTf, hashKeyTf);
    AddPublisherThread(tf_bridge_ptr, bind(&Base::GenerateTF, this, std::placeholders::_1));
  }

  auto callback_sub = [this,
      data_bridge_ptr](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
      const auto msgBuf = MakeControlMessage(msg);
      SetBufferToSimulator(data_bridge_ptr, msgBuf);
    };

  // ROS2 Subscriber
  sub_micom_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(), callback_sub);

  srv_reset_odom_ = create_service<std_srvs::srv::Empty>(
    "reset_odometry", std::bind(&Micom::ResetOdometryCallback, this, _1, _2, _3));
}

void Micom::ResetOdometryCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  RequestReplyMessage(info_bridge_ptr, "reset_odometry");
}

string Micom::MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  cloisim::msgs::Twist twistBuf;  // m/s and rad/s
  auto linear_ptr = twistBuf.mutable_linear();
  auto angular_ptr = twistBuf.mutable_angular();
  linear_ptr->set_x(msg->linear.x);
  linear_ptr->set_y(msg->linear.y);
  linear_ptr->set_z(msg->linear.z);
  angular_ptr->set_x(msg->angular.x);
  angular_ptr->set_y(msg->angular.y);
  angular_ptr->set_z(msg->angular.z);

  // DBG_SIM_INFO("%f %f", linear_ptr->x(), angular_ptr->z());
  // m/s velocity input
  // double vel_left_wheel = (vel_lin - (vel_rot * (0.50f) / 2.0));
  // double vel_right_wheel = (vel_lin + (vel_rot * (0.50f) / 2.0));
  // const auto vel_rot_wheel = (0.5f * vel_rot * wheel_tread);
  // auto lin_vel_left_wheel = vel_lin - vel_rot_wheel; // m/s
  // auto lin_vel_right_wheel = vel_lin + vel_rot_wheel; // m/s

  string message;
  twistBuf.SerializeToString(&message);
  return message;
}

void Micom::PublishData(const string & buffer)
{
  if (!pb_micom_.ParseFromString(buffer)) {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_micom_.time());

  // DBG_SIM_WRN("Simulation time %u %u size(%d)",
  //             pb_micom_.time().sec(), pb_micom_.time().nsec(), bufferLength);

  UpdateOdom();
  UpdateImu();
  UpdateBattery();

  // publish data
  PublishTF(odom_tf_);
  pub_odom_->publish(msg_odom_);
  pub_imu_->publish(msg_imu_);
  pub_battery_->publish(msg_battery_);
}

void Micom::UpdateOdom()
{
  if (!pb_micom_.has_odom()) {
    return;
  }

  msg_odom_.header.stamp = GetTime();
  msg::Convert(pb_micom_.odom().pose(), msg_odom_.pose.pose.position);
  msg_odom_.pose.pose.position.z = 0.0;  // position.z contians yaw value

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0.0, 0.0, pb_micom_.odom().pose().z());
  geometry_msgs::msg::Convert(tf2_q, msg_odom_.pose.pose.orientation);

  // static int cnt = 0;
  // if (cnt++ % LOGGING_PERIOD == 0)
  // {
  //   DBG_SIM_MSG(
  //       "Wheel odom x[%+3.5f] y[%+3.5f] theta[%+3.5f] vel[lin(%+3.5f) ang(%+3.5f)] time(%f)",
  //       msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y,
  //       pb_micom_.odom().pose().z(),
  //       msg_odom_.twist.twist.linear.x, msg_odom_.twist.twist.angular.z, step_time);
  // }

  msg::Convert(pb_micom_.odom().twist_linear(), msg_odom_.twist.twist.linear);
  msg::Convert(pb_micom_.odom().twist_angular(), msg_odom_.twist.twist.angular);

  // Update TF
  odom_tf_.header.stamp = msg_odom_.header.stamp;
  geometry_msgs::msg::Convert(msg_odom_.pose.pose.position, odom_tf_.transform.translation);
  odom_tf_.transform.rotation = msg_odom_.pose.pose.orientation;
}

void Micom::UpdateImu()
{
  if (!pb_micom_.has_imu()) {
    return;
  }

  msg_imu_.header.stamp = GetTime();

  msg::Convert(pb_micom_.imu().orientation(), msg_imu_.orientation);
  msg::Convert(pb_micom_.imu().angular_velocity(), msg_imu_.angular_velocity);
  msg::Convert(pb_micom_.imu().linear_acceleration(), msg_imu_.linear_acceleration);

  std::fill(begin(msg_imu_.orientation_covariance), end(msg_imu_.orientation_covariance), 0.0);
  std::fill(
    begin(msg_imu_.angular_velocity_covariance), end(msg_imu_.angular_velocity_covariance), 0.0);
  std::fill(
    begin(msg_imu_.linear_acceleration_covariance), end(msg_imu_.linear_acceleration_covariance),
    0.0);
}

void Micom::UpdateBattery()
{
  if (pb_micom_.has_battery()) {
    msg_battery_.header.stamp = GetTime();
    msg_battery_.voltage = pb_micom_.battery().voltage();
    msg_battery_.current = 0.0;
  }
}

}  // namespace cloisim_ros
