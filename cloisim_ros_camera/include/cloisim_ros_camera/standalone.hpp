/**
 *  @file   standalone.hpp
 *  @date   2024-03-08
 *  @author Hyunseok Yang
 *  @brief
 *        rclcpp init for standalone
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2024 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_CAMERA__STANDALONE_HPP_
#define CLOISIM_ROS_CAMERA__STANDALONE_HPP_

#include <memory>
#include <string>

#include <cloisim_ros_bringup_param/bringup_param.hpp>

template<class T>
static void run_standalone_single_executor(
  const std::string target_node_name, const std::string target_parts_name)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>(target_node_name);
  bringup_param_node->TargetPartsType(target_parts_name);

  executor.add_node(bringup_param_node);

  const auto filtered_result = bringup_param_node->GetBringUpList(true);
  if (filtered_result.empty()) {
    return;
  }

  rclcpp::NodeOptions node_options;
  bringup_param_node->StoreFilteredInfoAsParameters(filtered_result, node_options);

  const auto is_single_mode = bringup_param_node->IsSingleMode();
  const auto model_name = bringup_param_node->TargetModel();
  const auto node_name = bringup_param_node->TargetPartsName();

  auto node = (is_single_mode) ? std::make_shared<T>(node_options, node_name) :
    std::make_shared<T>(node_options, node_name, model_name);

  executor.add_node(node);
  executor.spin();
}

#endif  // CLOISIM_ROS_CAMERA__STANDALONE_HPP_
