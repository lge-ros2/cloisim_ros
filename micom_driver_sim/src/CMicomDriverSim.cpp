/**
 *  @file   CMicomDriversim.cpp
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

#include "micom_driver_sim/CMicomDriverSim.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "protobuf/param.pb.h"

#define MM2M(X) ((X)*0.001)
#define LOGGING_PERIOD 200

using namespace std;
using namespace chrono_literals;

CMicomDriverSim::CMicomDriverSim()
    : DriverSim("micom_driver_sim"),
      m_hashKeyPub(""),
      m_hashKeySub(""),
      wheel_base(0.0),
      wheel_radius(0.0),
      m_use_pub(true),
      m_use_sub(true)
{
  msg_odom.header.frame_id = "odom";
  msg_odom.child_frame_id = "base_footprint";
  msg_imu.header.frame_id = "imu_link";

  Start();
}

CMicomDriverSim::~CMicomDriverSim()
{
  Stop();
}

void CMicomDriverSim::Initialize()
{
  string input_part_name;
  string sensor_part_name;
  vector<double> transform_base;
  vector<double> transform_imu;
  vector<double> transform_wheelLeft;
  vector<double> transform_wheelRight;

  get_parameter_or("sim.parts_tx", input_part_name, string("MICOM_INPUT"));
  get_parameter_or("sim.parts_rx", sensor_part_name, string("MICOM_SENSOR"));
  get_parameter_or("transform.base", transform_base, vector<double>({0, 0, 0, 0, 0, 0}));
  get_parameter_or("transform.imu", transform_imu, vector<double>({0, 0, 0, 0, 0, 0}));
  get_parameter_or("transform.wheel.left", transform_wheelLeft, vector<double>({0, 0, 0, 0, 0, 0}));
  get_parameter_or("transform.wheel.right", transform_wheelRight, vector<double>({0, 0, 0, 0, 0, 0}));
  get_parameter_or("wheel.base", wheel_base, 449.0);
  get_parameter_or("wheel.radius", wheel_radius, 95.5);

  m_hashKeyPub = GetRobotName() + input_part_name;
  m_hashKeySub = GetRobotName() + sensor_part_name;

  DBG_SIM_INFO("[CONFIG] sim.parts_tx:%s,", input_part_name.c_str());
  DBG_SIM_INFO("[CONFIG] sim.parts_rx:%s,", sensor_part_name.c_str());
  DBG_SIM_INFO("[CONFIG] wheel.base:%f", wheel_base);
  DBG_SIM_INFO("[CONFIG] wheel.radius:%f", wheel_radius);

  DBG_SIM_INFO("Hash Key sub(%s) pub(%s)", m_hashKeySub.c_str(), m_hashKeyPub.c_str());

  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = 0.0;
  q_msg.y = 0.0;
  q_msg.z = 0.0;
  q_msg.w = 1.0;

  geometry_msgs::msg::TransformStamped base_tf;
  base_tf.header.frame_id = "base_footprint";
  base_tf.child_frame_id = "base_link";
  base_tf.transform.translation.x = transform_base[0];
  base_tf.transform.translation.y = transform_base[1];
  base_tf.transform.translation.z = transform_base[2];
  base_tf.transform.rotation = q_msg;
  AddStaticTf2(base_tf);

  geometry_msgs::msg::TransformStamped imu_tf;
  imu_tf.header.frame_id = base_tf.child_frame_id;
  imu_tf.child_frame_id = "imu_link";
  imu_tf.transform.translation.x = transform_imu[0];
  imu_tf.transform.translation.y = transform_imu[1];
  imu_tf.transform.translation.z = transform_imu[2];
  imu_tf.transform.rotation = q_msg;
  AddStaticTf2(imu_tf);

  odom_tf.header.frame_id = msg_odom.header.frame_id;
  odom_tf.child_frame_id = msg_odom.child_frame_id;
  odom_tf.transform.translation.x = 0;
  odom_tf.transform.translation.y = 0;
  odom_tf.transform.translation.z = 0;
  odom_tf.transform.rotation = q_msg;

  wheel_left_tf.header.frame_id = "base_link";
  wheel_left_tf.child_frame_id = "wheel_left_link";
  wheel_left_tf.transform.translation.x = transform_wheelLeft[0];
  wheel_left_tf.transform.translation.y = transform_wheelLeft[1];
  wheel_left_tf.transform.translation.z = transform_wheelLeft[2];
  wheel_left_tf.transform.rotation = q_msg;

  wheel_right_tf.header.frame_id = "base_link";
  wheel_right_tf.child_frame_id = "wheel_right_link";
  wheel_right_tf.transform.translation.x = transform_wheelRight[0];
  wheel_right_tf.transform.translation.y = transform_wheelRight[1];
  wheel_right_tf.transform.translation.z = transform_wheelRight[2];
  wheel_right_tf.transform.rotation = q_msg;

  if (m_use_sub)
  {
    GetSimBridge()->Connect(SimBridge::Mode::SUB, m_hashKeySub);
  }

  if (m_use_pub)
  {
    GetSimBridge()->Connect(SimBridge::Mode::PUB, m_hashKeyPub);
  }

  // ROS2 Publisher
  pubBatteryState = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SensorDataQoS());
  pubOdometry = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  pubImu = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());

  auto callback_sub = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
    const string msgBuf = MakeControlMessage(msg);
    MicomWrite(msgBuf.data(), msgBuf.size());
  };

  // ROS2 Subscriber
  subMicom = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), callback_sub);
}

void CMicomDriverSim::Deinitialize()
{
  GetSimBridge()->Disconnect();
}

string CMicomDriverSim::MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  auto vel_lin = msg->linear.x * 1000.0; // mm/s
  auto vel_rot = msg->angular.z;         // rad/s

  // mm/s velocity input
  // double vel_left_wheel = (vel_lin - (vel_rot * (0.50f * 1000.0) / 2.0));
  // double vel_right_wheel = (vel_lin + (vel_rot * (0.50f * 1000.0) / 2.0));
  const double vel_rot_wheel = (0.5f * vel_rot * wheel_base);
  double vel_left_wheel = vel_lin - vel_rot_wheel;
  double vel_right_wheel = vel_lin + vel_rot_wheel;

  gazebo::msgs::Param writeBuf;
  gazebo::msgs::Any *pVal;

  writeBuf.set_name("control_type");
  pVal = writeBuf.mutable_value();
  pVal->set_type(gazebo::msgs::Any::INT32);
  pVal->set_int_value(1);

  gazebo::msgs::Param *const pLinearVel = writeBuf.add_children();
  pLinearVel->set_name("nLeftWheelVelocity");
  pVal = pLinearVel->mutable_value();
  pVal->set_type(gazebo::msgs::Any::INT32);
  pVal->set_int_value(vel_left_wheel);

  gazebo::msgs::Param *const pAngularVel = writeBuf.add_children();
  pAngularVel->set_name("nRightWheelVelocity");
  pVal = pAngularVel->mutable_value();
  pVal->set_type(gazebo::msgs::Any::INT32);
  pVal->set_int_value(vel_right_wheel);

  string message = "";
  writeBuf.SerializeToString(&message);
  return message;
}

void CMicomDriverSim::MicomWrite(const void *const pcBuf, const uint32_t unSize)
{
  if (pcBuf != nullptr && unSize > 0)
  {
    GetSimBridge()->Send(pcBuf, unSize);
  }
}

void CMicomDriverSim::UpdateData()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (IsRunThread())
  {
    const bool succeeded = GetSimBridge()->Receive(&pBuffer, bufferLength, false);

    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

      // try reconnection
      if (IsRunThread())
      {
        GetSimBridge()->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
        GetSimBridge()->Reconnect(SimBridge::Mode::PUB, m_hashKeyPub);
      }

      continue;
    }

    if (!m_pbBufMicom.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    m_simTime = rclcpp::Time(m_pbBufMicom.time().sec(), m_pbBufMicom.time().nsec());

    //DBG_SIM_WRN("Simulation time %u %u size(%d)",
    //  m_pbBufMicom.time().sec(), m_pbBufMicom.time().nsec(), bufferLength);

    // reset odom info when sim time is reset
    if (m_pbBufMicom.time().sec() == 0 && m_pbBufMicom.time().nsec() < 50000000)
    {
      DBG_SIM_WRN("Simulation time has been reset!!!");
      odom_pose.fill(0.0);
      odom_vel.fill(0.0);
      last_rad.fill(0.0);
    }

#if 0
    static int cnt = 0;
    if (cnt++ % LOGGING_PERIOD == 0)
    {
      DBG_SIM_INFO("recv [%06d][%d][%d][%f]",
          cnt,
          m_pbBufMicom.odom().speed_left(), m_pbBufMicom.odom().speed_right(),
          m_pbBufMicom.imu().angular_velocity().z());
    }
#endif

    UpdateOdom();
    UpdateImu();
    UpdateBattery();

    PublishTF();

    // publish data
    pubOdometry->publish(msg_odom);
    pubImu->publish(msg_imu);
    pubBatteryState->publish(msg_battery);
  }
}

bool CMicomDriverSim::CalculateOdometry(
    const rclcpp::Duration duration,
    const double _wheel_left,
    const double _wheel_right,
    const double _theta)
{
  static const double M_2PI = M_PI * 2;
  const double step_time = duration.seconds();

  if (step_time <= 0.0000000f)
    return false;

  double delta_theta = 0.0f;
  static double last_theta = 0.0f;

  double v = 0.0f; // v = translational velocity [m/s]
  double w = 0.0f; // w = rotational velocity [rad/s]

   // rotation value of wheel [rad]
  double wheel_l = _wheel_left * step_time;
  double wheel_r = _wheel_right * step_time;

  if (isnan(wheel_l))
    wheel_l = 0.0f;

  if (isnan(wheel_r))
    wheel_r = 0.0f;

  // origin: delta_s = wheel_radius * (wheel_r + wheel_l) / 2.0f;
  const double delta_s = (wheel_r + wheel_l) / 2.0f;

  delta_theta = _theta - last_theta;

  if (delta_theta > M_PI)
    delta_theta -= M_2PI;

  if (delta_theta < -M_PI)
    delta_theta += M_2PI;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0f));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0f));
  odom_pose[2] += delta_theta;

  if (odom_pose[2] > M_2PI)
    odom_pose[2] -= M_2PI;

  if (odom_pose[2] < -M_2PI)
    odom_pose[2] += M_2PI;

  // compute odometric instantaneouse velocity
  // origin: v = delta_s / step_time;
  v = (_wheel_right + _wheel_left) / 2.0f;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_theta = _theta;
  last_rad[0] += wheel_l;
  last_rad[1] += wheel_r;

  return true;
}

void CMicomDriverSim::UpdateOdom()
{
  if (!m_pbBufMicom.has_odom() || !m_pbBufMicom.has_imu())
  {
    return;
  }

  // get wheel linear velocity mm/s
  const int16_t nSpeedLeft_MM = m_pbBufMicom.odom().speed_left();
  const int16_t nSpeedRight_MM = m_pbBufMicom.odom().speed_right();
  // DBG_SIM_MSG("nSpeedLeft: %d, nSpeedRight: %d, imu.x: %f, imu.y: %f, imu.z: %f, imu.w: %f",
  //         nSpeedLeft_MM, nSpeedRight_MM, m_pbBufMicom.imu().orientation().x(), m_pbBufMicom.imu().orientation().y(),
  //         m_pbBufMicom.imu().orientation().z(), m_pbBufMicom.imu().orientation().w());
  // update velocity m/s
  const double fWheelVelLeft_M = MM2M((float)nSpeedLeft_MM / WHEEL_RADIUS_RATIO);
  const double fWheelVelRight_M = MM2M((float)nSpeedRight_MM / WHEEL_RADIUS_RATIO);

  const double wheel_left = fWheelVelLeft_M;
  const double wheel_right = fWheelVelRight_M;

  const auto orientation = m_pbBufMicom.imu().orientation();
  const double theta = atan2f(2 * ((orientation.w() * orientation.z()) + (orientation.x() * orientation.y())),
                              1 - 2 * ((orientation.y() * orientation.y()) + (orientation.z() * orientation.z())));

  static rclcpp::Time last_time = m_simTime;
  const rclcpp::Duration duration(m_simTime.nanoseconds() - last_time.nanoseconds());

  CalculateOdometry(duration, wheel_left, wheel_right, theta);

  static int cnt = 0;
  if (cnt++ % LOGGING_PERIOD == 0)
  {
    DBG_SIM_MSG("Wheel odom x[%f] y[%f] theta[%f] vel_lin[%f] vel_ang[%f]",
                odom_pose[0], odom_pose[1], odom_pose[2],
                odom_vel[0], odom_pose[2]);
  }

  geometry_msgs::msg::Quaternion *q_msg;

  msg_odom.pose.pose.position.x = odom_pose[0];
  msg_odom.pose.pose.position.y = odom_pose[1];
  msg_odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_pose[2]);
  q_msg = &msg_odom.pose.pose.orientation;
  q_msg->x = q.x();
  q_msg->y = q.y();
  q_msg->z = q.z();
  q_msg->w = q.w();

  msg_odom.twist.twist.linear.x = odom_vel[0];
  msg_odom.twist.twist.angular.z = odom_vel[2];

  // Update TF
  odom_tf.header.stamp = m_simTime;
  odom_tf.transform.translation.x = msg_odom.pose.pose.position.x;
  odom_tf.transform.translation.y = msg_odom.pose.pose.position.y;
  odom_tf.transform.translation.z = msg_odom.pose.pose.position.z;
  odom_tf.transform.rotation = msg_odom.pose.pose.orientation;
  AddTf2(odom_tf);

  wheel_left_tf.header.stamp = m_simTime;
  q_msg = &wheel_left_tf.transform.rotation;
  q.setRPY(0, last_rad[0], 0);
  q_msg->x = q.x();
  q_msg->y = q.y();
  q_msg->z = q.z();
  q_msg->w = q.w();
  AddTf2(wheel_left_tf);

  wheel_right_tf.header.stamp = m_simTime;
  q_msg = &wheel_right_tf.transform.rotation;
  q.setRPY(0, last_rad[1], 0);
  q_msg->x = q.x();
  q_msg->y = q.y();
  q_msg->z = q.z();
  q_msg->w = q.w();
  AddTf2(wheel_right_tf);

  last_time = m_simTime;
}

void CMicomDriverSim::UpdateImu()
{
  msg_imu.header.stamp = m_simTime;
  // 	imuLink.rotation.eulerAngles.ToString("F4"), imuInitialRotation.

  msg_imu.orientation.x = m_pbBufMicom.imu().orientation().x();
  msg_imu.orientation.y = m_pbBufMicom.imu().orientation().y();
  msg_imu.orientation.z = m_pbBufMicom.imu().orientation().z();
  msg_imu.orientation.w = m_pbBufMicom.imu().orientation().w();

  msg_imu.orientation_covariance[0] = 0.0;
  msg_imu.orientation_covariance[1] = 0.0;
  msg_imu.orientation_covariance[2] = 0.0;
  msg_imu.orientation_covariance[3] = 0.0;
  msg_imu.orientation_covariance[4] = 0.0;
  msg_imu.orientation_covariance[5] = 0.0;
  msg_imu.orientation_covariance[6] = 0.0;
  msg_imu.orientation_covariance[7] = 0.0;
  msg_imu.orientation_covariance[8] = 0.0;

  msg_imu.angular_velocity.x = m_pbBufMicom.imu().angular_velocity().x();
  msg_imu.angular_velocity.y = m_pbBufMicom.imu().angular_velocity().y();
  msg_imu.angular_velocity.z = m_pbBufMicom.imu().angular_velocity().z();

  msg_imu.angular_velocity_covariance[0] = 0.0;
  msg_imu.angular_velocity_covariance[1] = 0.0;
  msg_imu.angular_velocity_covariance[2] = 0.0;
  msg_imu.angular_velocity_covariance[3] = 0.0;
  msg_imu.angular_velocity_covariance[4] = 0.0;
  msg_imu.angular_velocity_covariance[5] = 0.0;
  msg_imu.angular_velocity_covariance[6] = 0.0;
  msg_imu.angular_velocity_covariance[7] = 0.0;
  msg_imu.angular_velocity_covariance[8] = 0.0;

  msg_imu.linear_acceleration.x = m_pbBufMicom.imu().linear_acceleration().x();
  msg_imu.linear_acceleration.y = m_pbBufMicom.imu().linear_acceleration().y();
  msg_imu.linear_acceleration.z = m_pbBufMicom.imu().linear_acceleration().z();

  msg_imu.linear_acceleration_covariance[0] = 0.0;
  msg_imu.linear_acceleration_covariance[1] = 0.0;
  msg_imu.linear_acceleration_covariance[2] = 0.0;
  msg_imu.linear_acceleration_covariance[3] = 0.0;
  msg_imu.linear_acceleration_covariance[4] = 0.0;
  msg_imu.linear_acceleration_covariance[5] = 0.0;
  msg_imu.linear_acceleration_covariance[6] = 0.0;
  msg_imu.linear_acceleration_covariance[7] = 0.0;
  msg_imu.linear_acceleration_covariance[8] = 0.0;
}

void CMicomDriverSim::UpdateBattery()
{
  msg_battery.header.stamp = m_simTime;
  msg_battery.voltage = 0.0;
  msg_battery.current = 0.0;
}
