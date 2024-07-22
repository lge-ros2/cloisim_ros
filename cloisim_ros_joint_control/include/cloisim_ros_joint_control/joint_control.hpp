/**
 *  @file   joint_control.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 JointControl class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#ifndef CLOISIM_ROS_JOINT_CONTROL__JOINT_CONTROL_HPP_
#define CLOISIM_ROS_JOINT_CONTROL__JOINT_CONTROL_HPP_

#include <map>
#include <string>

#include <cloisim_ros_base/base.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

namespace cloisim_ros
{
class JointControl : public Base
{
 public:
  explicit JointControl(
      const rclcpp::NodeOptions &options_,
      const std::string node_name, const std::string namespace_ = "");
  explicit JointControl(const std::string namespace_ = "");
  virtual ~JointControl();

 private:
  void Initialize() override;
  void Deinitialize() override{};

 private:
  void PublishData(const std::string &buffer);

  void JointControlWrite(zmq::Bridge *const bridge_ptr, const std::string &buffer);

  std::string MakeCommandMessage(control_msgs::msg::JointJog::ConstSharedPtr msg);

  void GetRobotDescription(zmq::Bridge *const bridge_ptr);

 private:
  zmq::Bridge *info_bridge_ptr;

  std::map<std::string, std::string> target_transform_name;

  std_msgs::msg::String msg_description_;

  // ROS2 JointControl publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;

  // ROS2 Joint Jog subscriber
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr sub_joint_job_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_desc_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_JOINT_CONTROL__JOINT_CONTROL_HPP_
