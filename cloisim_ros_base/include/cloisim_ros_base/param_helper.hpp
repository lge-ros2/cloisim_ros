/**
 *  @file   param_helper.hpp
 *  @date   2026-04-24
 *  @author Hyunseok Yang
 *  @brief
 *        Helper functions for the new Param proto3 API
 *        (migrated from proto2 name/value to proto3 params map)
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_BASE__PARAM_HELPER_HPP_
#define CLOISIM_ROS_BASE__PARAM_HELPER_HPP_

#include <cloisim_msgs/param.pb.h>

#include <string>

namespace cloisim_ros
{
namespace param
{

/// Get the first key from a Param's params map (replaces old name())
static inline std::string GetName(const cloisim::msgs::Param & param)
{
  if (param.params_size() > 0) {
    return param.params().begin()->first;
  }
  return "";
}

/// Get the value for a specific key from a Param's params map
static inline const cloisim::msgs::Any & GetValue(
  const cloisim::msgs::Param & param, const std::string & key)
{
  static const cloisim::msgs::Any empty;
  auto it = param.params().find(key);
  if (it != param.params().end()) {
    return it->second;
  }
  return empty;
}

/// Get the first value from a Param's params map (replaces old value())
static inline const cloisim::msgs::Any & GetValue(const cloisim::msgs::Param & param)
{
  static const cloisim::msgs::Any empty;
  if (param.params_size() > 0) {
    return param.params().begin()->second;
  }
  return empty;
}

/// Check if a Param has a specific key
static inline bool HasKey(const cloisim::msgs::Param & param, const std::string & key)
{
  return param.params().count(key) > 0;
}

/// Check if a Param has any params (replaces old has_value() / has_name())
static inline bool HasValue(const cloisim::msgs::Param & param)
{
  return param.params_size() > 0;
}

/// Set a name-value pair on a Param (replaces old set_name() + mutable_value())
static inline void Set(
  cloisim::msgs::Param & param, const std::string & name, const cloisim::msgs::Any & value)
{
  (*param.mutable_params())[name] = value;
}

/// Set a name with empty value on a Param (replaces old set_name())
static inline void SetName(cloisim::msgs::Param & param, const std::string & name)
{
  (*param.mutable_params())[name] = cloisim::msgs::Any();
}

/// Get a string from the header data map (replaces old header().str_id())
static inline std::string GetHeaderDataValue(
  const cloisim::msgs::Header & header, const std::string & key)
{
  for (const auto & data : header.data()) {
    if (data.key() == key && data.value_size() > 0) {
      return data.value(0);
    }
  }
  return "";
}

}  // namespace param
}  // namespace cloisim_ros

#endif  // CLOISIM_ROS_BASE__PARAM_HELPER_HPP_
