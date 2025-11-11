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
#include <cloisim_msgs/joystick.pb.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_micom/micom.hpp"

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

  auto base_link_name = std::string("base_link");
  info_bridge_ptr_ = CreateBridge();
  if (info_bridge_ptr_ != nullptr) {
    info_bridge_ptr_->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);
    SetStaticTransforms(info_bridge_ptr_);

    GetRos2Parameter(info_bridge_ptr_);

    base_link_name = GetFrameId("base_link");
    DBG_SIM_INFO("base_link_name(%s)", base_link_name.c_str());
  }

  const auto parent_frame_id = "base_footprint";
  auto base_link_pose = IdentityPose();
  base_link_pose.set_name(base_link_name);

  SetStaticTf2(base_link_pose, parent_frame_id);

  pub_battery_ =
    create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SensorDataQoS());

  pub_bumper_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("bumper", rclcpp::SensorDataQoS());

  pub_bumper_states_ =
    create_publisher<cloisim_ros_msgs::msg::ContactsArray>(
    "bumper/contacts",
    rclcpp::SensorDataQoS());

  pub_ir_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("ir", rclcpp::SensorDataQoS());

  pub_ir_pose_ =
    create_publisher<geometry_msgs::msg::PoseArray>("ir/pose", rclcpp::SensorDataQoS());

  pub_uss_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("uss", rclcpp::SensorDataQoS());

  pub_uss_pose_ =
    create_publisher<geometry_msgs::msg::PoseArray>("uss/pose", rclcpp::SensorDataQoS());

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
    AddBridgeReceiveWorker(data_bridge_ptr, bind(&Micom::PublishData, this, std::placeholders::_1));
  }

  auto tf_bridge_ptr = CreateBridge();
  if (tf_bridge_ptr != nullptr) {
    tf_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portTf, hashKeyTf);
    AddBridgeReceiveWorker(tf_bridge_ptr, bind(&Base::GenerateTF, this, std::placeholders::_1));
  }

  auto callback_sub_cmdvel = [this,
      data_bridge_ptr](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
      const auto msgBuf = MakeControlMessage(msg);
      SetBufferToSimulator(data_bridge_ptr, msgBuf);
    };

  auto callback_sub_joy = [this,
      data_bridge_ptr](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
      const auto msgBuf = MakeControlMessage(msg);
      SetBufferToSimulator(data_bridge_ptr, msgBuf);
    };

  auto callback_sub_mowing_blade_height = [this,
      data_bridge_ptr](const std_msgs::msg::Float32::SharedPtr msg) -> void {
      const auto msgBuf = MakeMowingBladeHeightMessage(msg);
      SetBufferToSimulator(data_bridge_ptr, msgBuf);
    };

  auto callback_sub_mowing_rev_speed = [this,
      data_bridge_ptr](const std_msgs::msg::UInt16::SharedPtr msg) -> void {
      const auto msgBuf = MakeMowingRevSpeedMessage(msg);
      SetBufferToSimulator(data_bridge_ptr, msgBuf);
    };

  // ROS2 Subscriber
  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(), callback_sub_cmdvel);

  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(), callback_sub_joy);

  sub_blade_height_ = create_subscription<std_msgs::msg::Float32>(
    "mowing/blade/height", rclcpp::ServicesQoS(), callback_sub_mowing_blade_height);

  sub_blade_rev_speed_ = create_subscription<std_msgs::msg::UInt16>(
    "mowing/blade/rev_speed", rclcpp::ServicesQoS(), callback_sub_mowing_rev_speed);

  srv_reset_odom_ = create_service<std_srvs::srv::Empty>(
    "reset_odometry", std::bind(&Micom::ResetOdometryCallback, this, _1, _2, _3));
}

void Micom::ResetOdometryCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  RequestReplyMessage(info_bridge_ptr_, "reset_odometry");
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

string Micom::MakeControlMessage(const sensor_msgs::msg::Joy::SharedPtr msg) const
{
  cloisim::msgs::Joystick joyBuf;

  auto translation_ptr = joyBuf.mutable_translation();
  auto rotation_ptr = joyBuf.mutable_rotation();

  // std::cout << "msg Axis=";
  // for (const auto & msg_axis : msg->axes) std::cout << msg_axis << ", ";
  // std::cout << std::endl;
  const auto roll = msg->axes[0];
  const auto linear_x = msg->axes[1];
  const auto yaw = msg->axes[2];  // angular
  const auto pitch = msg->axes[3];
  const auto linear_z = ((msg->axes[4] + 1.0) - (msg->axes[5] + 1.0)) * 0.5;

  translation_ptr->set_x(linear_x);
  translation_ptr->set_y(0);
  translation_ptr->set_z(linear_z);
  rotation_ptr->set_x(roll);
  rotation_ptr->set_y(pitch);
  rotation_ptr->set_z(yaw);

  // std::cout << "msg Button=";
  for (const auto & msg_button : msg->buttons) {
    joyBuf.add_buttons(msg_button);
    // std::cout << msg_button << ", ";
  }
  // std::cout << std::endl;

  string message;
  joyBuf.SerializeToString(&message);
  return message;
}

string Micom::MakeMowingBladeHeightMessage(const std_msgs::msg::Float32::SharedPtr msg) const
{
  cloisim::msgs::Param paramBuf;

  paramBuf.set_name("mowing_blade_height");
  auto pVal = paramBuf.mutable_value();
  pVal->set_type(cloisim::msgs::Any::DOUBLE);
  pVal->set_double_value(msg->data);

  string message;
  paramBuf.SerializeToString(&message);
  return message;
}

string Micom::MakeMowingRevSpeedMessage(const std_msgs::msg::UInt16::SharedPtr msg) const
{
  cloisim::msgs::Param paramBuf;

  paramBuf.set_name("mowing_blade_rev_speed");
  auto pVal = paramBuf.mutable_value();
  pVal->set_type(cloisim::msgs::Any::INT32);
  pVal->set_int_value(msg->data);

  string message;
  paramBuf.SerializeToString(&message);
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
  UpdateBumper();
  UpdateIR();
  UpdateUSS();

  // publish data
  if (pb_micom_.has_odom()) {
    PublishTF(odom_tf_);
    pub_odom_->publish(msg_odom_);
  }

  pub_imu_->publish(msg_imu_);
  pub_battery_->publish(msg_battery_);
  pub_bumper_->publish(msg_bumper_);
  pub_bumper_states_->publish(msg_bumper_contacts_array_);
  pub_uss_->publish(msg_uss_);
  pub_uss_pose_->publish(msg_uss_pose_array_);
  pub_ir_->publish(msg_ir_);
  pub_ir_pose_->publish(msg_ir_pose_array_);
}

void Micom::UpdateOdom()
{
  if (!pb_micom_.has_odom()) {
    return;
  }

  const auto & odom = pb_micom_.odom();

  msg_odom_.header.stamp = GetTime();
  msg::Convert(odom.pose(), msg_odom_.pose.pose.position);
  msg_odom_.pose.pose.position.z = 0.0;  // position.z contians yaw value

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0.0, 0.0, odom.pose().z());
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
  if (odom.has_twist()) {
    const auto & twist = odom.twist();
    msg::Convert(twist.linear(), msg_odom_.twist.twist.linear);
    msg::Convert(twist.angular(), msg_odom_.twist.twist.angular);
  }

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

  msg_imu_.header.stamp = msg::Convert(pb_micom_.imu().stamp());

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

void Micom::UpdateBumper()
{
  msg_bumper_.data.clear();
  msg_bumper_contacts_array_.contacts.clear();

  msg_bumper_.data.resize(pb_micom_.bumper_size());
  msg_bumper_contacts_array_.header.stamp = GetTime();
  msg_bumper_contacts_array_.contacts.resize(pb_micom_.bumper_size());

  if (pb_micom_.bumper_size() > 0) {
    for (auto i = 0; i < pb_micom_.bumper_size(); i++) {
      const auto & bumper = pb_micom_.bumper(i);

      msg_bumper_.data[i] = bumper.bumped();

      if (bumper.has_contacts()) {
        const auto & contacts = bumper.contacts();

        ros_gz_interfaces::msg::Contacts ros_gz_contacts;
        msg::Convert(contacts, ros_gz_contacts);
        ros_gz_contacts.header.frame_id = "bumper_link";

        msg_bumper_contacts_array_.contacts[i] = ros_gz_contacts;
      }
    }
  }
}

void Micom::UpdateIR()
{
  msg_ir_.data.clear();
  msg_ir_pose_array_.poses.clear();
  if (pb_micom_.ir_size()) {
    msg_ir_.data.resize(pb_micom_.ir_size());
    msg_ir_pose_array_.poses.resize(pb_micom_.ir_size());

    msg_ir_pose_array_.header.stamp = GetTime();
    msg_ir_pose_array_.header.frame_id = "ir_link";

    for (auto i = 0; i < pb_micom_.ir_size(); i++) {
      const auto & ir = pb_micom_.ir(i);

      msg_ir_.data[i] = ir.distance();

      if (ir.has_state()) {
        geometry_msgs::msg::Pose p;
        msg::Convert(ir.state().world_pose(), p);
        msg_ir_pose_array_.poses.push_back(p);
      }
    }
    // std::cout << "ir Size " <<  pb_micom_.ir_size() << std::endl;
  }
}

void Micom::UpdateUSS()
{
  msg_uss_.data.clear();
  if (pb_micom_.uss_size()) {
    msg_uss_.data.resize(pb_micom_.uss_size());

    for (auto i = 0; i < pb_micom_.uss_size(); i++) {
      const auto & uss = pb_micom_.uss(i);

      msg_uss_.data[i] = uss.distance();

      if (uss.has_state()) {
        geometry_msgs::msg::Pose p;
        msg::Convert(uss.state().world_pose(), p);
        msg_uss_pose_array_.poses.push_back(p);
      }
    }
    // std::cout << "ir Size " <<  pb_micom_.ir_size() << std::endl;
  }
}

}  // namespace cloisim_ros
