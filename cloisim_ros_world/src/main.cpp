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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_world");
  executor.add_node(bringup_param_node);
  bringup_param_node->IsSingleMode(true);
  bringup_param_node->TargetPartsType("WORLD");

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("single_mode", bool(bringup_param_node->IsSingleMode()));

  const auto result = bringup_param_node->GetBringUpList(true);

  std::shared_ptr<cloisim_ros::Base> node = nullptr;
  if (!result.empty())
  {
    // std::cout<< result << std::endl;
    const auto model_name = result.begin().key().asString();
    const auto isResultEmpty = result[model_name].empty();
    const auto first_iteration = (isResultEmpty) ? Json::ValueConstIterator() : result[model_name].begin();
    const auto node_name = (isResultEmpty) ? bringup_param_node->TargetPartsName() : first_iteration.key().asString();
    const auto bridge_infos = (isResultEmpty) ? Json::Value() : *first_iteration;
    // std::cout << "model_name: " << model_name << std::endl;
    // std::cout << "node_name: " << node_name << std::endl;
    // std::cout << bridge_infos << std::endl;

    node_options.append_parameter_override("model", (bringup_param_node->TargetModel().empty()) ? model_name : bringup_param_node->TargetModel());

    cloisim_ros::BringUpParam::StoreBridgeInfosAsParameters(bridge_infos, node_options);
    node = std::make_shared<cloisim_ros::World>(node_options, node_name);
  }

  if (node != nullptr)
  {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}