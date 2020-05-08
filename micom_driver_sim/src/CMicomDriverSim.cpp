/**
 *  @file   CMicomDriversim.cpp
 *  @date   2019-03-28
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
#include <unistd.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include "protobuf/param.pb.h"

#define MM2M(X) ((X)*0.001)

using namespace std::chrono_literals;

CMicomDriverSim::CMicomDriverSim()
  : Node("micom_driver_sim", rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  , m_bMicomThreadFlag(false)
  , m_fThreadMicomReadPeriodMs(50.0) // --> 20hz
  , m_hashKeyPub("")
  , m_hashKeySub("")
  , wheel_base(0.0)
  , wheel_radius(0.0)
  , m_use_pub(true)
  , m_use_sub(true)
{
  node_handle = std::shared_ptr<::rclcpp::Node>(this);

  this->tf_broadcaster
    = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle);

  this->static_tf_broadcaster
    = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle);

  std::string sim_ip("");
  int sim_manager_port(0);
  std::string robot_name;
  std::string input_part_name;
  std::string sensor_part_name;

  get_parameter("sim.ip_address", sim_ip);
  get_parameter("sim.manager_port", sim_manager_port);
  get_parameter_or("wheel.base", wheel_base, 449.0);
  get_parameter_or("wheel.radius", wheel_radius, 95.5);
  get_parameter_or("sim.model", robot_name, std::string("CLOI_porter"));
  get_parameter_or("sim.parts_tx", input_part_name, std::string("MICOM_INPUT"));
  get_parameter_or("sim.parts_rx", sensor_part_name, std::string("MICOM_SENSOR"));

  DBG_SIM_INFO("[CONFIG] sim manage ip:%s, port:%d", sim_ip.c_str(), sim_manager_port);

  m_hashKeyPub = robot_name + input_part_name;
  m_hashKeySub = robot_name + sensor_part_name;

  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());
  DBG_SIM_INFO("hash Key pub: %s", m_hashKeyPub.c_str());

  m_pSimBridge = new SimBridge();
  DBG_SIM_INFO("[CONFIG] wheel.base:%f .. ", wheel_base);
  DBG_SIM_INFO("[CONFIG] wheel.radius:%f ..", wheel_radius);
  DBG_SIM_INFO("[CONFIG] sim.model:%s ..", robot_name.c_str());
  DBG_SIM_INFO("[CONFIG] sim.parts_tx:%s, ..", input_part_name.c_str());
  DBG_SIM_INFO("[CONFIG] sim.parts_rx:%s, ..", sensor_part_name.c_str());

  if (m_pSimBridge)
  {
    m_pSimBridge->SetSimMasterAddress(sim_ip);
    m_pSimBridge->SetPortManagerPort(sim_manager_port);
  }

  // ROS2 Publisher
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  pubBatteryStateMsg = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SystemDefaultsQoS());

  pubOdometryMsg = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  pubImuMsg = this->create_publisher<sensor_msgs::msg::Imu>("imu", qos);

  Start();
}

CMicomDriverSim::~CMicomDriverSim()
{
  m_bMicomThreadFlag = false;
  usleep(100);

  Stop();
  delete m_pSimBridge;
}

void CMicomDriverSim::Start()
{
  if (m_pSimBridge)
  {
    if(this->m_use_sub)
    {
      m_pSimBridge->Connect(SimBridge::Mode::SUB, m_hashKeySub);
      m_bMicomThreadFlag = true;
      m_thread = std::thread([=]() { MicomProc(); });
    }

    if (this->m_use_pub)
    {
      m_pSimBridge->Connect(SimBridge::Mode::PUB, m_hashKeyPub);
    }
  }

  auto callback_pub_tf_static = [this]() -> void
    {
      UpdateStaticTF(m_SimTime);
    };

  auto callback_sub = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      const std::string strMsgBuf = MakeControlMessage(msg);
      MicomWrite(strMsgBuf.data(), strMsgBuf.size());
    };


  // ROS2 timer for updating TF Publisher
  timer_tf_static = this->create_wall_timer(1s, callback_pub_tf_static);

  // ROS2 Subscriber
  subMobileMCUMsg = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
    rclcpp::SystemDefaultsQoS(), callback_sub);

}

void CMicomDriverSim::Stop()
{
  m_bMicomThreadFlag = false;
  usleep(100);

  if (m_thread.joinable())
  {
    m_thread.join();
  }

  m_pSimBridge->Disconnect();
}

std::string CMicomDriverSim::MakeControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  double vel_lin = msg->linear.x * 1000.0; // mm/s
  double vel_rot = msg->angular.z;         // rad/s

  //  mm/s velocity input
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

  std::string message = "";
  writeBuf.SerializeToString(&message);
  return message;

}

void CMicomDriverSim::MicomProc()
{
  DBG_SIM_MSG("MicomProc Thread start");
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (m_bMicomThreadFlag)
  {
    const auto clockStart = std::chrono::high_resolution_clock::now();

    const bool succeeded = m_pSimBridge->Receive(&pBuffer, bufferLength, false);

    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s",
              bufferLength, zmq_strerror(zmq_errno()));

      // try reconnection
      if (m_bMicomThreadFlag) {
        m_pSimBridge->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
        m_pSimBridge->Reconnect(SimBridge::Mode::PUB, m_hashKeyPub);
      }
      continue;
    }

    if (!m_MicomData.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    //DBG_SIM_WRN("Simulation time %u %u size(%d)",
    //  m_MicomData.time().sec(), m_MicomData.time().nsec(), bufferLength);

    // reset odom info when sim time is reset
    if (m_MicomData.time().sec() == 0 && m_MicomData.time().nsec() < 50000000)
    {
      DBG_SIM_WRN("Simulation time has been reset!!!");
      odom_pose.fill(0.0);
      odom_vel.fill(0.0);
      last_rad.fill(0.0);
    }

#if 0
    static int cnt = 0;
    if (cnt++ % 20 == 0)
    {
      DBG_SIM_INFO("recv [%06d][%d][%d][%f]",
          cnt,
          m_MicomData.odom().speed_left(),
          m_MicomData.odom().speed_right(),
          m_MicomData.imu().angular_velocity().z()
          );
    }
#endif

    m_SimTime = rclcpp::Time(m_MicomData.time().sec(), m_MicomData.time().nsec());

    UpdateOdom(m_SimTime);
    UpdateImu(m_SimTime);
    UpdateBattery(m_SimTime);
    UpdateTF(m_SimTime);

    // publish data
    pubOdometryMsg->publish(msg_odom);
    pubImuMsg->publish(msg_imu);
    pubBatteryStateMsg->publish(msg_battery);

    const auto clockEnd = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::milli> elapsed = clockEnd - clockStart;

    const double target_sleep_time
      = (m_fThreadMicomReadPeriodMs< elapsed.count())?
        0.0:(m_fThreadMicomReadPeriodMs - elapsed.count());

    //std::cout << "target sleep time: " << target_sleep_time << ", elapsed: " << elapsed.count()<<  std::endl;
    std::this_thread::sleep_for(
        std::chrono::nanoseconds((uint32_t)(target_sleep_time * 1000000.0)));
  }
}

bool CMicomDriverSim::MicomWrite(const void* const pcBuf, const uint32_t unSize)
{
  if (pcBuf == nullptr)
    return false;

  m_pSimBridge->Send(pcBuf, unSize);
  usleep(1);
  return true;
}

bool CMicomDriverSim::CalculateOdometry(
    const rclcpp::Duration duration,
    const double _wheel_left,
    const double _wheel_right,
    const double _theta)
{
  const double step_time = duration.seconds();

  if (step_time <= 0.0000000f)
    return false;

  double wheel_l = 0.0f;
  double wheel_r = 0.0f; // rotation value of wheel [rad]

  double delta_s = 0.0f;
  double delta_theta = 0.0f;
  static double last_theta = 0.0f;

  double v = 0.0f; // v = translational velocity [m/s]
  double w = 0.0f; // w = rotational velocity [rad/s]

  wheel_l = _wheel_left * step_time;
  wheel_r = _wheel_right * step_time;

  if (std::isnan(wheel_l))
    wheel_l = 0.0f;

  if (std::isnan(wheel_r))
    wheel_r = 0.0f;

  // origin: delta_s = wheel_radius * (wheel_r + wheel_l) / 2.0f;
  delta_s = (wheel_r + wheel_l) / 2.0f;

  const double M_2PI = M_PI * 2;
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

void CMicomDriverSim::UpdateOdom(const rclcpp::Time timestamp)
{
  if (!m_MicomData.has_odom() || !m_MicomData.has_imu())
    return;

  // get wheel linear velocity mm/s
  const int16_t nSpeedLeft_MM = m_MicomData.odom().speed_left();
  const int16_t nSpeedRight_MM = m_MicomData.odom().speed_right();
  // DBG_SIM_MSG("nSpeedLeft: %d, nSpeedRight: %d, imu.x: %f, imu.y: %f, imu.z: %f, imu.w: %f",
  //         nSpeedLeft_MM, nSpeedRight_MM, m_MicomData.imu().orientation().x(), m_MicomData.imu().orientation().y(),
  //         m_MicomData.imu().orientation().z(), m_MicomData.imu().orientation().w());
  // update velocity m/s
  const double fWheelVelLeft_M  = MM2M((float)nSpeedLeft_MM / WHEEL_RADIUS_RATIO);
  const double fWheelVelRight_M = MM2M((float)nSpeedRight_MM / WHEEL_RADIUS_RATIO);

  const double wheel_left = fWheelVelLeft_M;
  const double wheel_right = fWheelVelRight_M;

  const auto orientation = m_MicomData.imu().orientation();
  const double theta = atan2f(2 * ((orientation.w() * orientation.z()) + (orientation.x() * orientation.y())),
                              1 - 2 * ((orientation.y()* orientation.y()) + (orientation.z() * orientation.z())));

  static rclcpp::Time last_time = timestamp;
  const rclcpp::Duration duration(timestamp.nanoseconds() - last_time.nanoseconds());

  CalculateOdometry(duration, wheel_left, wheel_right, theta);

  static int cnt = 0;
  if (cnt++ % 100 == 0)
  {
    DBG_SIM_MSG("Wheel odom x[%f] y[%f] theta[%f] vel_lin[%f] vel_ang[%f]",
        odom_pose[0], odom_pose[1], odom_pose[2],
        odom_vel[0], odom_pose[2]);
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_pose[2]);

  msg_odom.header.stamp = timestamp;
  msg_odom.header.frame_id = "odom";
  msg_odom.child_frame_id = "base_footprint";

  msg_odom.pose.pose.position.x = odom_pose[0];
  msg_odom.pose.pose.position.y = odom_pose[1];
  msg_odom.pose.pose.position.z = 0.0;

  msg_odom.pose.pose.orientation.x = q.x();
  msg_odom.pose.pose.orientation.y = q.y();
  msg_odom.pose.pose.orientation.z = q.z();
  msg_odom.pose.pose.orientation.w = q.w();

  msg_odom.twist.twist.linear.x  = odom_vel[0];
  msg_odom.twist.twist.angular.z = odom_vel[2];

  last_time = timestamp;
}

void CMicomDriverSim::UpdateImu(const rclcpp::Time timestamp)
{
  //rclcpp::Time _timestamp(m_MicomData.imu().stamp().sec(),
  //                        m_MicomData.imu().stamp().nsec());

  msg_imu.header.stamp =  timestamp;
			// 	imuLink.rotation.eulerAngles.ToString("F4"), imuInitialRotation.
  msg_imu.header.frame_id = "imu_link";

  msg_imu.orientation.x = m_MicomData.imu().orientation().x();
  msg_imu.orientation.y = m_MicomData.imu().orientation().y();
  msg_imu.orientation.z = m_MicomData.imu().orientation().z();
  msg_imu.orientation.w = m_MicomData.imu().orientation().w();

  msg_imu.orientation_covariance[0] = 0.0;
  msg_imu.orientation_covariance[1] = 0.0;
  msg_imu.orientation_covariance[2] = 0.0;
  msg_imu.orientation_covariance[3] = 0.0;
  msg_imu.orientation_covariance[4] = 0.0;
  msg_imu.orientation_covariance[5] = 0.0;
  msg_imu.orientation_covariance[6] = 0.0;
  msg_imu.orientation_covariance[7] = 0.0;
  msg_imu.orientation_covariance[8] = 0.0;

  msg_imu.angular_velocity.x = m_MicomData.imu().angular_velocity().x();
  msg_imu.angular_velocity.y = m_MicomData.imu().angular_velocity().y();
  msg_imu.angular_velocity.z = m_MicomData.imu().angular_velocity().z();

  msg_imu.angular_velocity_covariance[0] = 0.0;
  msg_imu.angular_velocity_covariance[1] = 0.0;
  msg_imu.angular_velocity_covariance[2] = 0.0;
  msg_imu.angular_velocity_covariance[3] = 0.0;
  msg_imu.angular_velocity_covariance[4] = 0.0;
  msg_imu.angular_velocity_covariance[5] = 0.0;
  msg_imu.angular_velocity_covariance[6] = 0.0;
  msg_imu.angular_velocity_covariance[7] = 0.0;
  msg_imu.angular_velocity_covariance[8] = 0.0;

  msg_imu.linear_acceleration.x = m_MicomData.imu().linear_acceleration().x();
  msg_imu.linear_acceleration.y = m_MicomData.imu().linear_acceleration().y();
  msg_imu.linear_acceleration.z = m_MicomData.imu().linear_acceleration().z();

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

void CMicomDriverSim::UpdateBattery(const rclcpp::Time timestamp)
{
  msg_battery.header.stamp = timestamp;
  msg_battery.voltage = 0.0;
  msg_battery.current = 0.0;
}

void CMicomDriverSim::UpdateTF(const rclcpp::Time timestamp)
{
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = timestamp;
  odom_tf.header.frame_id = msg_odom.header.frame_id;
  odom_tf.child_frame_id = msg_odom.child_frame_id;
  odom_tf.transform.translation.x = msg_odom.pose.pose.position.x;
  odom_tf.transform.translation.y = msg_odom.pose.pose.position.y;
  odom_tf.transform.translation.z = msg_odom.pose.pose.position.z;
  odom_tf.transform.rotation = msg_odom.pose.pose.orientation;

  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;

  q.setRPY(0, last_rad[0], 0);
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  geometry_msgs::msg::TransformStamped wheel_left_tf;
  wheel_left_tf.header.stamp = timestamp;
  wheel_left_tf.header.frame_id = "base_link";
  wheel_left_tf.child_frame_id = "wheel_left_link";
  wheel_left_tf.transform.translation.x = 0.001;
  wheel_left_tf.transform.translation.y = 0.225;
  wheel_left_tf.transform.translation.z = 0.07;
  wheel_left_tf.transform.rotation = q_msg;

  q.setRPY(0, last_rad[1], 0);
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  geometry_msgs::msg::TransformStamped wheel_right_tf;
  wheel_right_tf.header.stamp = timestamp;
  wheel_right_tf.header.frame_id = "base_link";
  wheel_right_tf.child_frame_id = "wheel_right_link";
  wheel_right_tf.transform.translation.x = 0.001;
  wheel_right_tf.transform.translation.y = -0.225;
  wheel_right_tf.transform.translation.z = 0.07;
  wheel_right_tf.transform.rotation = q_msg;

  tf_broadcaster->sendTransform(odom_tf);
  tf_broadcaster->sendTransform(wheel_left_tf);
  tf_broadcaster->sendTransform(wheel_right_tf);
}

void CMicomDriverSim::UpdateStaticTF(const rclcpp::Time timestamp)
{
  geometry_msgs::msg::Quaternion fixed_rot;
  fixed_rot.x = 0.0;
  fixed_rot.y = 0.0;
  fixed_rot.z = 0.0;
  fixed_rot.w = 1.0;

  geometry_msgs::msg::TransformStamped base_tf;
  base_tf.header.stamp = timestamp;
  base_tf.header.frame_id = "base_footprint";
  base_tf.child_frame_id = "base_link";
  base_tf.transform.translation.x = 0.0;
  base_tf.transform.translation.y = 0.0;
  base_tf.transform.translation.z = 0.01;
  base_tf.transform.rotation = fixed_rot;

  geometry_msgs::msg::TransformStamped imu_tf;
  imu_tf.header.stamp = timestamp;
  imu_tf.header.frame_id = base_tf.child_frame_id;
  imu_tf.child_frame_id = "imu_link";
  imu_tf.transform.translation.x = 0.0;
  imu_tf.transform.translation.y = 0.0;
  imu_tf.transform.translation.z = 0.19;
  imu_tf.transform.rotation = fixed_rot;

  static_tf_broadcaster->sendTransform(base_tf);
  static_tf_broadcaster->sendTransform(imu_tf);
}