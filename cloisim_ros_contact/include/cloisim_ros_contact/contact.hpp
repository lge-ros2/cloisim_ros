/**
 *  @file   contact.hpp
 *  @date   2025-01-23
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Contact class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2025 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_CONTACT__CONTACT_HPP_
#define CLOISIM_ROS_CONTACT__CONTACT_HPP_

#include <cloisim_msgs/contacts.pb.h>

#include <string>

#include <cloisim_ros_base/base.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

namespace cloisim_ros
{
class Contact : public Base
{
public:
  explicit Contact(
    const rclcpp::NodeOptions & options_, const std::string node_name,
    const std::string namespace_ = "");
  explicit Contact(const std::string namespace_ = "");
  ~Contact();

private:
  void Initialize() override;
  void Deinitialize() override {}

private:
  void Convert();
  void PublishData(const std::string & buffer);

private:
  // buffer from simulation
  cloisim::msgs::Contacts pb_buf_;

  // Contact msgs
  gazebo_msgs::msg::ContactsState msg_contacts_state_;

  // publisher
  rclcpp::Publisher<gazebo_msgs::msg::ContactsState>::SharedPtr pub_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_CONTACT__CONTACT_HPP_
