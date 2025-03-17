/**
 *  @file   helper.hpp
 *  @date   2025-02-11
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CLOiSim-ROS helper
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_BASE__HELPER_HPP_
#define CLOISIM_ROS_BASE__HELPER_HPP_

#include <cloisim_msgs/contacts.pb.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <limits>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <ros_gz_interfaces/msg/joint_wrench.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace cloisim_ros
{
struct Quaternion
{
  double w, x, y, z;

  Quaternion inverse() const
  {
    Quaternion q {this->w, this->x, this->y, this->z};
    const auto s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

    if (std::abs(s) < std::numeric_limits<double>::epsilon()) {
      q.w = 1.0;
      q.x = 0.0;
      q.y = 0.0;
      q.z = 0.0;
    } else {
      // deal with non-normalized quaternion
      // div by s so q * qinv = identity
      q.w /= s;
      q.x /= -s;
      q.y /= -s;
      q.z /= -s;
    }
    return q;
  }

  Quaternion operator*(const Quaternion & q) const
  {
    return {
      w * q.w - x * q.x - y * q.y - z * q.z,  // w
        w * q.x + x * q.w + y * q.z - z * q.y,  // x
        w * q.y + y * q.w + z * q.x - x * q.z,  // y
        w * q.z + z * q.w + x * q.y - y * q.x   // z
    };
  }

  cloisim::msgs::Vector3d rotateVectorReverse(const cloisim::msgs::Vector3d & vec)
  {
    const Quaternion tmp = {0, vec.x(), vec.y(), vec.z()};
    const auto q_result = this->inverse() * (tmp * (*this));

    cloisim::msgs::Vector3d result;
    result.set_x(q_result.x);
    result.set_y(q_result.y);
    result.set_z(q_result.z);
    return result;
  }
};

namespace msg
{
static rclcpp::Time Convert(const cloisim::msgs::Time & time)
{
  return rclcpp::Time(time.sec(), time.nsec());
}

static void Convert(const cloisim::msgs::Vector3d & src, geometry_msgs::msg::Vector3 & dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

static void Convert(const cloisim::msgs::Vector3d & src, geometry_msgs::msg::Point32 & dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

static void Convert(const cloisim::msgs::Vector3d & src, geometry_msgs::msg::Point & dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

static void Convert(const cloisim::msgs::Quaternion & src, geometry_msgs::msg::Quaternion & dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  dst.w = src.w();
}

static void Convert(const cloisim::msgs::Pose & src, geometry_msgs::msg::Pose & dst)
{
  Convert(src.position(), dst.position);
  Convert(src.orientation(), dst.orientation);
}

static void Convert(const cloisim::msgs::Contacts & src, ros_gz_interfaces::msg::Contacts & dst)
{
  dst.header.stamp = Convert(src.time());
  dst.contacts.clear();

  Quaternion identityQuat{1, 0, 0, 0};
  const auto contactSize = src.contact_size();
  for (auto i = 0; i < contactSize; i++) {
    const auto contact = src.contact(i);

    // For each collision contact
    ros_gz_interfaces::msg::Contact state;
    state.collision1.name = contact.collision1();
    state.collision2.name = contact.collision2();

    state.wrenches.clear();
    state.positions.clear();
    state.normals.clear();
    state.depths.clear();

    const auto contactGroupSize = contact.position_size();
    for (auto j = 0; j < contactGroupSize; ++j) {
      // Get force, torque and rotate into user specified frame.
      // identity is default for now if world is used.
      const auto wrench = contact.wrench(j);

      const auto body1forceVec = wrench.body_1_wrench().force();
      const auto body1force = identityQuat.rotateVectorReverse(body1forceVec);
      const auto body2forceVec = wrench.body_2_wrench().force();
      const auto body2force = identityQuat.rotateVectorReverse(body2forceVec);

      // TODO(hyunseok-yang): not support in CLOiSim
      // const auto torqueVec = wrench.body_1_wrench().torque();
      // const auto torque = identityQuat.rotateVectorReverse(torqueVec);

      // set wrenches
      ros_gz_interfaces::msg::JointWrench jointWrench;
      jointWrench.body_1_name.data = wrench.body_1_name();
      jointWrench.body_1_id.data = wrench.body_1_id();
      jointWrench.body_2_name.data = wrench.body_2_name();
      jointWrench.body_2_id.data = wrench.body_2_id();
      jointWrench.body_1_wrench.force.x = body1force.x();
      jointWrench.body_1_wrench.force.y = body1force.y();
      jointWrench.body_1_wrench.force.z = body1force.z();
      // jointWrench.body_1_wrench.torque.x = torque.x();
      // jointWrench.body_1_wrench.torque.y = torque.y();
      // jointWrench.body_1_wrench.torque.z = torque.z();
      state.wrenches.push_back(jointWrench);

      // transform contact positions into relative frame
      const auto positionVec = contact.position(j);
      const auto position = identityQuat.rotateVectorReverse(positionVec);  // - frame_pos;

      geometry_msgs::msg::Vector3 contact_position;
      contact_position.x = position.x();
      contact_position.y = position.y();
      contact_position.z = position.z();
      state.positions.push_back(contact_position);

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      const auto normalVec = contact.normal(j);
      const auto normal = identityQuat.rotateVectorReverse(normalVec);

      // set contact normals
      geometry_msgs::msg::Vector3 contact_normal;
      contact_normal.x = normal.x();
      contact_normal.y = normal.y();
      contact_normal.z = normal.z();
      state.normals.push_back(contact_normal);

      state.depths.push_back(contact.depth(j));
    }

    dst.contacts.push_back(state);
  }
}
}  // namespace msg
}  // namespace cloisim_ros

namespace geometry_msgs
{
namespace msg
{
static void Convert(const tf2::Quaternion & src, Quaternion & dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  dst.w = src.w();
}

static void Convert(const Point & src, Vector3 & dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}
}  // namespace msg
}  // namespace geometry_msgs

#endif  // CLOISIM_ROS_BASE__HELPER_HPP_
