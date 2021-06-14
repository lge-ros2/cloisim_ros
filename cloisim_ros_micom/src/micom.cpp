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

#include "cloisim_ros_micom/micom.hpp"
#include <cloisim_ros_base/helper.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/twist.pb.h>

// #define LOGGING_PERIOD 1000

using namespace std;
using namespace chrono_literals;
using namespace placeholders;
using namespace cloisim;
using namespace cloisim_ros;

Micom::Micom(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
{
  Start();
}

Micom::Micom(const string namespace_)
    : Micom(rclcpp::NodeOptions(), "cloisim_ros_micom", namespace_)
{
}

Micom::~Micom()
{
  Stop();
}

void Micom::Initialize()
{
  uint16_t portInfo, portTx, portRx;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));
  get_parameter_or("bridge.Tx", portTx, uint16_t(0));
  get_parameter_or("bridge.Rx", portRx, uint16_t(0));
  const auto hashKeyInfo = GetTargetHashKey("Info");
  const auto hashKeyPub = GetTargetHashKey("Rx");
  const auto hashKeySub = GetTargetHashKey("Tx");
  DBG_SIM_INFO("hash Key: info(%s) pub_(%s) sub(%s)", hashKeyInfo.c_str(), hashKeyPub.c_str(), hashKeySub.c_str());

  msg_imu_.header.frame_id = "imu_link";
  msg_odom_.header.frame_id = "odom";
  msg_odom_.child_frame_id = "base_footprint";

  SetTf2(odom_tf_, msg_odom_.child_frame_id, msg_odom_.header.frame_id);

  info_bridge_ptr = CreateBridge(hashKeyInfo);
  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetTransformNameInfo(info_bridge_ptr);

    std::string base_link_name("base_link");
    SetStaticTf2(base_link_name, "base_footprint");

    const auto transform_imu_name = target_transform_name["imu"];
    const auto transform_imu = GetObjectTransform(info_bridge_ptr, transform_imu_name);
    SetStaticTf2(transform_imu, transform_imu_name + "_link", base_link_name);

    const auto transform_wheel_0_name = target_transform_name["wheels/left"];
    const auto transform_wheel_0 = GetObjectTransform(info_bridge_ptr, transform_wheel_0_name);
    SetTf2(wheel_left_tf_, transform_wheel_0, transform_wheel_0_name + "_link", base_link_name);

    const auto init_left_q_msg = &wheel_left_tf_.transform.rotation;
    const auto wheel_left_quat = tf2::Quaternion(init_left_q_msg->x, init_left_q_msg->y, init_left_q_msg->z, init_left_q_msg->w);
    tf2::Matrix3x3(wheel_left_quat).getRPY(orig_left_wheel_rot_[0], orig_left_wheel_rot_[1], orig_left_wheel_rot_[2]);

    const auto transform_wheel_1_name = target_transform_name["wheels/right"];
    const auto transform_wheel_1 = GetObjectTransform(info_bridge_ptr, transform_wheel_1_name);
    SetTf2(wheel_right_tf_, transform_wheel_1, transform_wheel_1_name + "_link", base_link_name);

    const auto init_right_q_msg = &wheel_right_tf_.transform.rotation;
    const auto wheel_right_quat = tf2::Quaternion(init_right_q_msg->x, init_right_q_msg->y, init_right_q_msg->z, init_right_q_msg->w);
    tf2::Matrix3x3(wheel_right_quat).getRPY(orig_right_wheel_rot_[0], orig_right_wheel_rot_[1], orig_right_wheel_rot_[2]);
  }

  // ROS2 Publisher
  pub_battery_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SensorDataQoS());
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());

  auto pBridgeData = CreateBridge(hashKeyPub);
  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::PUB, portRx, hashKeyPub);
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portTx, hashKeySub);
    CreatePublisherThread(pBridgeData);
  }

  auto callback_sub = [this, pBridgeData](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
    const auto msgBuf = MakeControlMessage(msg);
    SetBufferToSimulator(pBridgeData, msgBuf);
  };

  // ROS2 Subscriber
  sub_micom_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), callback_sub);

  srv_reset_odom_ = create_service<std_srvs::srv::Empty>("reset_odometry", std::bind(&Micom::ResetOdometryCallback, this, _1, _2, _3));
}

void Micom::GetTransformNameInfo(zmq::Bridge* const bridge_ptr)
{
  if (bridge_ptr == nullptr)
  {
    return;
  }

  const auto reply = RequestReplyMessage(bridge_ptr, "request_transform_name");

  if (reply.IsInitialized() &&
      (reply.name().compare("ros2") == 0))
  {
    auto baseParam = reply.children(0);
    if (baseParam.IsInitialized() &&
        baseParam.name() == "transform_name")
    {
      auto param0 = baseParam.children(0);
      if (param0.name() == "imu" && param0.has_value() &&
          param0.value().type() == msgs::Any_ValueType_STRING &&
          !param0.value().string_value().empty())
      {
        target_transform_name["imu"] = param0.value().string_value();
      }

      auto param1 = baseParam.children(1);
      if (param1.name() == "wheels")
      {
        auto childParam0 = param1.children(0);
        if (childParam0.name() == "left" && childParam0.has_value() &&
            childParam0.value().type() == msgs::Any_ValueType_STRING &&
            !childParam0.value().string_value().empty())
        {
          target_transform_name["wheels/left"] = childParam0.value().string_value();
        }

        auto childParam1 = param1.children(1);
        if (childParam1.name() == "right" && childParam1.has_value() &&
            childParam1.value().type() == msgs::Any_ValueType_STRING &&
            !childParam1.value().string_value().empty())
        {
          target_transform_name["wheels/right"] = childParam1.value().string_value();
        }
      }
    }

    DBG_SIM_INFO("transform name imu:%s, wheels(0/1):%s/%s",
                 target_transform_name["imu"].c_str(),
                 target_transform_name["wheels/left"].c_str(),
                 target_transform_name["wheels/right"].c_str());
  }
}

void Micom::ResetOdometryCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  RequestReplyMessage(info_bridge_ptr, "reset_odometry");

  last_rad_.fill(0.0);
}

string Micom::MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  msgs::Twist twistBuf;  // m/s and rad/s
  auto linear_ptr = twistBuf.mutable_linear();
  auto angular_ptr = twistBuf.mutable_angular();
  linear_ptr->set_x(msg->linear.x);
  linear_ptr->set_y(msg->linear.y);
  linear_ptr->set_z(msg->linear.z);
  angular_ptr->set_x(msg->angular.x);
  angular_ptr->set_y(msg->angular.y);
  angular_ptr->set_z(msg->angular.z);

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

void Micom::UpdatePublishingData(const string &buffer)
{
  if (!pb_micom_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_micom_.time());

  //DBG_SIM_WRN("Simulation time %u %u size(%d)",
  //  pb_micom_.time().sec(), pb_micom_.time().nsec(), bufferLength);

  // reset odom info when sim time is reset
  if (GetTime().seconds() < DBL_EPSILON && GetTime().nanoseconds() < 50000000)
  {
    // DBG_SIM_WRN("Simulation time %u %u size(%d)", pb_micom_.time().sec(), pb_micom_.time().nsec(), bufferLength);
    DBG_SIM_WRN("Simulation time has been reset!!!");
    last_rad_.fill(0.0);
  }

#if 0
    static int cnt = 0;
    if (cnt++ % LOGGING_PERIOD == 0)
    {
      DBG_SIM_INFO("recv [%06d][%d][%d][%f]",
          cnt,
          pb_micom_.odom().speed_left(), pb_micom_.odom().speed_right(),
          pb_micom_.imu().angular_velocity().z());
    }
#endif

  UpdateOdom();
  UpdateImu();
  UpdateBattery();

  PublishTF();

  // publish data
  pub_odom_->publish(msg_odom_);
  pub_imu_->publish(msg_imu_);
  pub_battery_->publish(msg_battery_);
}

void Micom::UpdateOdom()
{
  if (!pb_micom_.has_odom() || !pb_micom_.has_imu())
  {
    return;
  }

  static rclcpp::Time last_time = GetTime();
  const rclcpp::Duration duration(GetTime().nanoseconds() - last_time.nanoseconds());
  last_time = GetTime();

  const auto step_time = duration.seconds();
  const auto wheel_anglular_vel_left = pb_micom_.odom().angular_velocity().left();
  const auto wheel_anglular_vel_right = pb_micom_.odom().angular_velocity().right();
  const auto wheel_l_circum = wheel_anglular_vel_left * step_time;
  const auto wheel_r_circum = wheel_anglular_vel_right * step_time;

  // DBG_SIM_MSG("nSpeedLeft: %d, nSpeedRight: %d, imu.x: %f, imu.y: %f, imu.z: %f, imu.w: %f",
  //         nSpeedLeft, nSpeedRight, pb_micom_.imu().orientation().x(), pb_micom_.imu().orientation().y(),
  //         pb_micom_.imu().orientation().z(), pb_micom_.imu().orientation().w());

  // circumference of wheel [rad] per step time.
  last_rad_[0] += wheel_l_circum;
  last_rad_[1] += wheel_r_circum;

  msg_odom_.header.stamp = GetTime();
  SetVector3MessageToGeometry(pb_micom_.odom().pose(), msg_odom_.pose.pose.position);
  msg_odom_.pose.pose.position.z = 0.0; // position.z contians yaw value
  SetVector3MessageToGeometry(pb_micom_.odom().twist_linear(), msg_odom_.twist.twist.linear);
  SetVector3MessageToGeometry(pb_micom_.odom().twist_angular(), msg_odom_.twist.twist.angular);

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0.0, 0.0, pb_micom_.odom().pose().z());
  SetTf2QuaternionToGeometry(tf2_q, msg_odom_.pose.pose.orientation);

  // static int cnt = 0;
  // if (cnt++ % LOGGING_PERIOD == 0)
  // {
  //   DBG_SIM_MSG("Wheel odom x[%+3.5f] y[%+3.5f] theta[%+3.5f] vel_lin[%+3.5f] vel_ang[%+3.5f] time(%f)",
  //               msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y, pb_micom_.odom().pose().z(),
  //               msg_odom_.twist.twist.linear.x, msg_odom_.twist.twist.angular.z, step_time);
  // }

  // Update TF
  odom_tf_.header.stamp = msg_odom_.header.stamp;
  SetPointToGeometry(msg_odom_.pose.pose.position, odom_tf_.transform.translation);
  odom_tf_.transform.rotation = msg_odom_.pose.pose.orientation;
  AddTf2(odom_tf_);

  wheel_left_tf_.header.stamp = msg_odom_.header.stamp;
  tf2_q.setRPY(orig_left_wheel_rot_[0], last_rad_[0], orig_left_wheel_rot_[2]);
  SetTf2QuaternionToGeometry(tf2_q, wheel_left_tf_.transform.rotation);
  AddTf2(wheel_left_tf_);

  wheel_right_tf_.header.stamp = msg_odom_.header.stamp;
  tf2_q.setRPY(orig_right_wheel_rot_[0], last_rad_[1], orig_right_wheel_rot_[2]);
  SetTf2QuaternionToGeometry(tf2_q, wheel_right_tf_.transform.rotation);
  AddTf2(wheel_right_tf_);
}

void Micom::UpdateImu()
{
  msg_imu_.header.stamp = GetTime();

  SetQuaternionMessageToGeometry(pb_micom_.imu().orientation(), msg_imu_.orientation);\

   // Fill covariances
  msg_imu_.orientation_covariance[0] = 0.0;
  msg_imu_.orientation_covariance[1] = 0.0;
  msg_imu_.orientation_covariance[2] = 0.0;
  msg_imu_.orientation_covariance[3] = 0.0;
  msg_imu_.orientation_covariance[4] = 0.0;
  msg_imu_.orientation_covariance[5] = 0.0;
  msg_imu_.orientation_covariance[6] = 0.0;
  msg_imu_.orientation_covariance[7] = 0.0;
  msg_imu_.orientation_covariance[8] = 0.0;

  SetVector3MessageToGeometry(pb_micom_.imu().angular_velocity(), msg_imu_.angular_velocity);

  msg_imu_.angular_velocity_covariance[0] = 0.0;
  msg_imu_.angular_velocity_covariance[1] = 0.0;
  msg_imu_.angular_velocity_covariance[2] = 0.0;
  msg_imu_.angular_velocity_covariance[3] = 0.0;
  msg_imu_.angular_velocity_covariance[4] = 0.0;
  msg_imu_.angular_velocity_covariance[5] = 0.0;
  msg_imu_.angular_velocity_covariance[6] = 0.0;
  msg_imu_.angular_velocity_covariance[7] = 0.0;
  msg_imu_.angular_velocity_covariance[8] = 0.0;

  SetVector3MessageToGeometry(pb_micom_.imu().linear_acceleration(), msg_imu_.linear_acceleration);

  msg_imu_.linear_acceleration_covariance[0] = 0.0;
  msg_imu_.linear_acceleration_covariance[1] = 0.0;
  msg_imu_.linear_acceleration_covariance[2] = 0.0;
  msg_imu_.linear_acceleration_covariance[3] = 0.0;
  msg_imu_.linear_acceleration_covariance[4] = 0.0;
  msg_imu_.linear_acceleration_covariance[5] = 0.0;
  msg_imu_.linear_acceleration_covariance[6] = 0.0;
  msg_imu_.linear_acceleration_covariance[7] = 0.0;
  msg_imu_.linear_acceleration_covariance[8] = 0.0;
}

void Micom::UpdateBattery()
{
  msg_battery_.header.stamp = GetTime();
  msg_battery_.voltage = 0.0;
  msg_battery_.current = 0.0;
}