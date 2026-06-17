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
#include <rclcpp/rclcpp.hpp>

// <model_name, node_type, node_name>, <is_added, do_remove, shared_ptr>
// The node is stored as the generic rclcpp::Node base so the same lifecycle map can hold
// both cloisim_ros device nodes and in-process robot_state_publisher nodes.
typedef std::map<
    std::tuple<std::string, std::string, std::string>,
    std::tuple<bool, bool, std::shared_ptr<rclcpp::Node>>>
  node_map_t;

// <model_name, node_type, node_name>
typedef std::vector<std::tuple<std::string, std::string, std::string>> loaded_key_list_t;

#endif  // CLOISIM_ROS_BRINGUP__TYPE_HPP_
