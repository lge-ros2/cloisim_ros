/**
 *  @file   main.cpp
 *  @date   2020-04-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 packages that helps to control unity simulation
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_world/world.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<cloisim_ros::Base> node = nullptr;
  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_world");
  bringup_param_node->IsSingleMode(true);
  bringup_param_node->TargetPartsType("WORLD");
  executor.add_node(bringup_param_node);

  const auto filtered_result = bringup_param_node->GetBringUpList(true);
  if (!filtered_result.empty()) {
    rclcpp::NodeOptions node_options;
    bringup_param_node->StoreFilteredInfoAsParameters(filtered_result, node_options);
    const auto node_name = bringup_param_node->TargetPartsName();
    node = std::make_shared<cloisim_ros::World>(node_options, node_name);
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
