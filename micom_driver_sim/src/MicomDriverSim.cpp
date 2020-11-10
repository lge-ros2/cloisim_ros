/**
 *  @file   MicomDriverSim.cpp
 *  @date   2020-05-25Z
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 MicomDriver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "micom_driver_sim/MicomDriverSim.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <protobuf/param.pb.h>

// #define LOGGING_PERIOD 1000

using namespace std;
using namespace chrono_literals;
using namespace gazebo;

MicomDriverSim::MicomDriverSim(const string node_name)
    : DriverSim(node_name, 2),
      m_hashKeyPub(""),
      m_hashKeySub(""),
      wheel_base(0.0),
      wheel_radius(0.0),
      base_link_name_("base_link"),
      use_pub_(true),
      use_sub_(true)
{
  last_rad_.fill(0.0);

  Start();
}

MicomDriverSim::~MicomDriverSim()
{
  Stop();
}

void MicomDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));
  get_parameter_or("bridge.Tx", portTx_, uint16_t(0));
  get_parameter_or("bridge.Rx", portRx_, uint16_t(0));
  // DBG_SIM_INFO("%d %d %d", portInfo, portTx_, portRx_);

  const auto hashKeyInfo = GetMainHashKey() + "Info";
  m_hashKeyPub = GetMainHashKey() + "Rx";
  m_hashKeySub = GetMainHashKey() + "Tx";
  DBG_SIM_INFO("Hash Key sub(%s) pub(%s)", m_hashKeySub.c_str(), m_hashKeyPub.c_str());

  msg_imu_.header.frame_id = "imu_link";
  msg_odom_.header.frame_id = "odom";
  msg_odom_.child_frame_id = "base_footprint";

  SetTf2(odom_tf_, msg_odom_.child_frame_id, msg_odom_.header.frame_id);

  SetupStaticTf2(base_link_name_, "base_footprint");

  auto pSimBridgeData = GetSimBridge(0);
  auto pSimBridgeInfo = GetSimBridge(1);

  if (pSimBridgeData != nullptr)
  {
    if (use_sub_)
    {
      pSimBridgeData->Connect(SimBridge::Mode::SUB, portTx_, m_hashKeySub);
    }

    if (use_pub_)
    {
      pSimBridgeData->Connect(SimBridge::Mode::PUB, portRx_, m_hashKeyPub);
    }
  }

  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetWeelInfo(pSimBridgeInfo);

    GetTransformNameInfo(pSimBridgeInfo);

    const auto transform_imu_name = target_transform_name["imu"];
    const auto transform_imu = GetObjectTransform(pSimBridgeInfo, transform_imu_name);
    SetupStaticTf2(transform_imu, transform_imu_name + "_link", base_link_name_);

    const auto transform_wheel_0_name = target_transform_name["wheels/left"];
    const auto transform_wheel_0 = GetObjectTransform(pSimBridgeInfo, transform_wheel_0_name);
    SetTf2(wheel_left_tf_, transform_wheel_0, transform_wheel_0_name + "_link", base_link_name_);

    const auto init_left_q_msg = &wheel_left_tf_.transform.rotation;
    const auto wheel_left_quat = tf2::Quaternion(init_left_q_msg->x, init_left_q_msg->y, init_left_q_msg->z, init_left_q_msg->w);
    tf2::Matrix3x3(wheel_left_quat).getRPY(orig_left_wheel_rot_[0], orig_left_wheel_rot_[1], orig_left_wheel_rot_[2]);

    const auto transform_wheel_1_name = target_transform_name["wheels/right"];
    const auto transform_wheel_1 = GetObjectTransform(pSimBridgeInfo, transform_wheel_1_name);
    SetTf2(wheel_right_tf_, transform_wheel_1, transform_wheel_1_name + "_link", base_link_name_);

    const auto init_right_q_msg = &wheel_right_tf_.transform.rotation;
    const auto wheel_right_quat = tf2::Quaternion(init_right_q_msg->x, init_right_q_msg->y, init_right_q_msg->z, init_right_q_msg->w);
    tf2::Matrix3x3(wheel_right_quat).getRPY(orig_right_wheel_rot_[0], orig_right_wheel_rot_[1], orig_right_wheel_rot_[2]);
  }

  // ROS2 Publisher
  pubBatteryState_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SensorDataQoS());
  pubOdometry_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  pubImu_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());

  auto callback_sub = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
    const auto msgBuf = MakeControlMessage(msg);
    MicomWrite(msgBuf.data(), msgBuf.size());
  };

  // ROS2 Subscriber
  subMicom_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), callback_sub);
}


void MicomDriverSim::Deinitialize()
{
  DisconnectSimBridges();
}

void MicomDriverSim::GetWeelInfo(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  request_msg.set_name("request_wheel_info");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pSimBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get wheel info, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromString(reply))
    {
      if (m_pbBufParam.IsInitialized() &&
          m_pbBufParam.name() == "wheelInfo")
      {
        auto baseParam = m_pbBufParam.children(0);
        if (baseParam.name() == "base" && baseParam.has_value())
        {
          wheel_base = baseParam.value().double_value();
        }

        auto sizeParam = m_pbBufParam.children(1);
        if (sizeParam.name() == "radius" && sizeParam.has_value())
        {
          wheel_radius = sizeParam.value().double_value();
        }
      }
    }
    else
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%ld)", reply.data(), reply.size());
    }
  }

  DBG_SIM_INFO("wheel.base:%f m", wheel_base);
  DBG_SIM_INFO("wheel.radius:%f m", wheel_radius);
}

void MicomDriverSim::GetTransformNameInfo(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;

  request_msg.set_name("request_ros2");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pSimBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get transform name info, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromString(reply) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%ld)", reply.data(), reply.size());
    }

    if (m_pbBufParam.IsInitialized() &&
        m_pbBufParam.name() == "ros2")
    {
      auto baseParam = m_pbBufParam.children(0);
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
}

string MicomDriverSim::MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  auto vel_lin = msg->linear.x;  // m/s
  auto vel_rot = msg->angular.z; // rad/s

  // m/s velocity input
  // double vel_left_wheel = (vel_lin - (vel_rot * (0.50f * 1000.0) / 2.0));
  // double vel_right_wheel = (vel_lin + (vel_rot * (0.50f * 1000.0) / 2.0));
  const auto vel_rot_wheel = (0.5f * vel_rot * wheel_base);
  auto lin_vel_left_wheel = vel_lin - vel_rot_wheel; // m/s
  auto lin_vel_right_wheel = vel_lin + vel_rot_wheel; // m/s

  msgs::Param writeBuf;
  msgs::Any *pVal;

  writeBuf.set_name("control_type");
  pVal = writeBuf.mutable_value();
  pVal->set_type(msgs::Any::INT32);
  pVal->set_int_value(1);

  auto const pLinearVel = writeBuf.add_children();
  pLinearVel->set_name("LeftWheelVelocity");
  pVal = pLinearVel->mutable_value();
  pVal->set_type(msgs::Any::DOUBLE);
  pVal->set_double_value(lin_vel_left_wheel);

  auto const pAngularVel = writeBuf.add_children();
  pAngularVel->set_name("RightWheelVelocity");
  pVal = pAngularVel->mutable_value();
  pVal->set_type(msgs::Any::DOUBLE);
  pVal->set_double_value(lin_vel_right_wheel);

  string message = "";
  writeBuf.SerializeToString(&message);
  return message;
}

void MicomDriverSim::MicomWrite(const void *const pcBuf, const uint32_t unSize)
{
  if (pcBuf != nullptr && unSize > 0)
  {
    GetSimBridge(0)->Send(pcBuf, unSize);
  }
}

void MicomDriverSim::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));
    return;
  }

  if (!pbBufMicom_.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBufMicom_.time().sec(), pbBufMicom_.time().nsec());

  //DBG_SIM_WRN("Simulation time %u %u size(%d)",
  //  pbBufMicom_.time().sec(), pbBufMicom_.time().nsec(), bufferLength);

  // reset odom info when sim time is reset
  if (m_simTime.seconds() < DBL_EPSILON && m_simTime.nanoseconds() < 50000000)
  {
    // DBG_SIM_WRN("Simulation time %u %u size(%d)", pbBufMicom_.time().sec(), pbBufMicom_.time().nsec(), bufferLength);
    DBG_SIM_WRN("Simulation time has been reset!!!");
    last_rad_.fill(0.0);
  }

#if 0
    static int cnt = 0;
    if (cnt++ % LOGGING_PERIOD == 0)
    {
      DBG_SIM_INFO("recv [%06d][%d][%d][%f]",
          cnt,
          pbBufMicom_.odom().speed_left(), pbBufMicom_.odom().speed_right(),
          pbBufMicom_.imu().angular_velocity().z());
    }
#endif

  UpdateOdom();
  UpdateImu();
  UpdateBattery();

  PublishTF();

  // publish data
  pubOdometry_->publish(msg_odom_);
  pubImu_->publish(msg_imu_);
  pubBatteryState_->publish(msg_battery_);
}

void MicomDriverSim::UpdateOdom()
{
  if (!pbBufMicom_.has_odom() || !pbBufMicom_.has_imu())
  {
    return;
  }

  static rclcpp::Time last_time = m_simTime;
  const rclcpp::Duration duration(m_simTime.nanoseconds() - last_time.nanoseconds());
  last_time = m_simTime;

  const auto step_time = duration.seconds();
  const auto wheel_anglular_vel_left = pbBufMicom_.odom().angular_velocity().left();
  const auto wheel_anglular_vel_right = pbBufMicom_.odom().angular_velocity().right();
  const auto wheel_l_circum = wheel_anglular_vel_left * step_time;
  const auto wheel_r_circum = wheel_anglular_vel_right * step_time;

  // DBG_SIM_MSG("nSpeedLeft: %d, nSpeedRight: %d, imu.x: %f, imu.y: %f, imu.z: %f, imu.w: %f",
  //         nSpeedLeft, nSpeedRight, pbBufMicom_.imu().orientation().x(), pbBufMicom_.imu().orientation().y(),
  //         pbBufMicom_.imu().orientation().z(), pbBufMicom_.imu().orientation().w());

  // circumference of wheel [rad] per step time.
  last_rad_[0] += wheel_l_circum;
  last_rad_[1] += wheel_r_circum;

  msg_odom_.header.stamp = m_simTime;
  msg_odom_.pose.pose.position.x = pbBufMicom_.odom().pose().x();
  msg_odom_.pose.pose.position.y = pbBufMicom_.odom().pose().y();
  msg_odom_.pose.pose.position.z = 0.0;

  msg_odom_.twist.twist.linear.x = pbBufMicom_.odom().twist_linear().x();
  msg_odom_.twist.twist.angular.z = pbBufMicom_.odom().twist_angular().z();

  geometry_msgs::msg::Quaternion *q_msg;
  tf2::Quaternion q;

  q.setRPY(0.0, 0.0, pbBufMicom_.odom().pose().z());
  q_msg = &msg_odom_.pose.pose.orientation;
  q_msg->x = q.x();
  q_msg->y = q.y();
  q_msg->z = q.z();
  q_msg->w = q.w();

  // static int cnt = 0;
  // if (cnt++ % LOGGING_PERIOD == 0)
  // {
  //   DBG_SIM_MSG("Wheel odom x[%+3.5f] y[%+3.5f] theta[%+3.5f] vel_lin[%+3.5f] vel_ang[%+3.5f] time(%f)",
  //               msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y, pbBufMicom_.odom().pose().z(),
  //               msg_odom_.twist.twist.linear.x, msg_odom_.twist.twist.angular.z, step_time);
  // }

  // Update TF
  odom_tf_.header.stamp = msg_odom_.header.stamp;
  odom_tf_.transform.translation.x = msg_odom_.pose.pose.position.x;
  odom_tf_.transform.translation.y = msg_odom_.pose.pose.position.y;
  odom_tf_.transform.translation.z = msg_odom_.pose.pose.position.z;
  odom_tf_.transform.rotation = msg_odom_.pose.pose.orientation;
  AddTf2(odom_tf_);

  wheel_left_tf_.header.stamp = msg_odom_.header.stamp;
  q.setRPY(orig_left_wheel_rot_[0], last_rad_[0], orig_left_wheel_rot_[2]);
  q_msg = &wheel_left_tf_.transform.rotation;
  q_msg->x = q.x();
  q_msg->y = q.y();
  q_msg->z = q.z();
  q_msg->w = q.w();
  AddTf2(wheel_left_tf_);

  wheel_right_tf_.header.stamp = msg_odom_.header.stamp;
  q.setRPY(orig_right_wheel_rot_[0], last_rad_[1], orig_right_wheel_rot_[2]);
  q_msg = &wheel_right_tf_.transform.rotation;
  q_msg->x = q.x();
  q_msg->y = q.y();
  q_msg->z = q.z();
  q_msg->w = q.w();
  AddTf2(wheel_right_tf_);
}

void MicomDriverSim::UpdateImu()
{
  msg_imu_.header.stamp = m_simTime;

  msg_imu_.orientation.x = pbBufMicom_.imu().orientation().x();
  msg_imu_.orientation.y = pbBufMicom_.imu().orientation().y();
  msg_imu_.orientation.z = pbBufMicom_.imu().orientation().z();
  msg_imu_.orientation.w = pbBufMicom_.imu().orientation().w();

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

  msg_imu_.angular_velocity.x = pbBufMicom_.imu().angular_velocity().x();
  msg_imu_.angular_velocity.y = pbBufMicom_.imu().angular_velocity().y();
  msg_imu_.angular_velocity.z = pbBufMicom_.imu().angular_velocity().z();

  msg_imu_.angular_velocity_covariance[0] = 0.0;
  msg_imu_.angular_velocity_covariance[1] = 0.0;
  msg_imu_.angular_velocity_covariance[2] = 0.0;
  msg_imu_.angular_velocity_covariance[3] = 0.0;
  msg_imu_.angular_velocity_covariance[4] = 0.0;
  msg_imu_.angular_velocity_covariance[5] = 0.0;
  msg_imu_.angular_velocity_covariance[6] = 0.0;
  msg_imu_.angular_velocity_covariance[7] = 0.0;
  msg_imu_.angular_velocity_covariance[8] = 0.0;

  msg_imu_.linear_acceleration.x = pbBufMicom_.imu().linear_acceleration().x();
  msg_imu_.linear_acceleration.y = pbBufMicom_.imu().linear_acceleration().y();
  msg_imu_.linear_acceleration.z = pbBufMicom_.imu().linear_acceleration().z();

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

void MicomDriverSim::UpdateBattery()
{
  msg_battery_.header.stamp = m_simTime;
  msg_battery_.voltage = 0.0;
  msg_battery_.current = 0.0;
}
