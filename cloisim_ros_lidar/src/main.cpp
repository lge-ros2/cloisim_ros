/**
 *  @file   main.cpp
 *  @date   2019-03-28
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Node that controls lidar sensor for simulation.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_lidar/lidar.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_lidar");
  bringup_param_node->TargetPartsType("LASER");
  executor.add_node(bringup_param_node);

  const auto filtered_result = bringup_param_node->GetBringUpList(true);

  std::shared_ptr<cloisim_ros::Base> node = nullptr;
  if (!filtered_result.empty()) {
    rclcpp::NodeOptions node_options;
    bringup_param_node->StoreFilteredInfoAsParameters(filtered_result, node_options);

    const auto is_single_mode = bringup_param_node->IsSingleMode();
    const auto model_name = bringup_param_node->TargetModel();
    const auto node_name = bringup_param_node->TargetPartsName();

    if (is_single_mode) {
      node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name, model_name);
    }
  }

  if (node != nullptr) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
