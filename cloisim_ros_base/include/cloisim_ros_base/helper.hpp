/**
 *  @file   helper.hpp
 *  @date   2021-01-14
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

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace cloisim_ros
{
namespace msg
{
static void Convert(
    const cloisim::msgs::Vector3d &src,
    geometry_msgs::msg::Vector3 &dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

static void Convert(
    const cloisim::msgs::Vector3d &src,
    geometry_msgs::msg::Point32 &dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

static void Convert(
    const cloisim::msgs::Vector3d &src,
    geometry_msgs::msg::Point &dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

static void Convert(
    const cloisim::msgs::Quaternion &src,
    geometry_msgs::msg::Quaternion &dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  dst.w = src.w();
}
}  // namespace msg
}  // namespace cloisim_ros

namespace geometry_msgs
{
namespace msg
{
static void Convert(
    const tf2::Quaternion &src,
    geometry_msgs::msg::Quaternion &dst)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  dst.w = src.w();
}

static void Convert(
    const geometry_msgs::msg::Point &src,
    geometry_msgs::msg::Vector3 &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}
}  // namespace msg
}  // namespace geometry_msgs

#endif  // CLOISIM_ROS_BASE__HELPER_HPP_
