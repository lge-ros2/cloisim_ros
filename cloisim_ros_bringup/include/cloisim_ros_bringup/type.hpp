/**
 *  @file   type.hpp
 *  @date   2024-07-22
 *  @author Hyunseok Yang
 *  @brief
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_BRINGUP__TYPE_HPP_
#define CLOISIM_ROS_BRINGUP__TYPE_HPP_

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <cloisim_ros_base/base.hpp>

// <model_name, node_type, node_name>, <is_added, do_remove, shared_ptr>
typedef std::map<
    std::tuple<std::string, std::string, std::string>,
    std::tuple<bool, bool, std::shared_ptr<cloisim_ros::Base>>>
  node_map_t;

// <model_name, node_type, node_name>
typedef std::vector<std::tuple<std::string, std::string, std::string>> loaded_key_list_t;

#endif  // CLOISIM_ROS_BRINGUP__TYPE_HPP_
