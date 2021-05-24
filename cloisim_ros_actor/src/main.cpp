/**
 *  @file   main.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 package for elevator system
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_actor/actor.hpp"
#include <cloisim_ros_bringup_param/bringup_param.hpp>

using namespace std::literals;

int main(int argc, char *argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::sleep_for(1000ms);

  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_actor");
  executor.add_node(bringup_param_node);
  bringup_param_node->IsSingleMode(true);
  bringup_param_node->TargetPartsType("ACTOR");

  const auto filtered_result = bringup_param_node->GetBringUpList(true);

  std::shared_ptr<cloisim_ros::Base> node = nullptr;
  if (!filtered_result.empty())
  {
    rclcpp::NodeOptions node_options;
    bringup_param_node->StoreFilteredInfoAsParameters(filtered_result, node_options);
    const auto node_name = bringup_param_node->TargetPartsName();
    node = std::make_shared<cloisim_ros::Actor>(node_options, node_name);
  }

  if (node != nullptr)
  {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();

  return 0;
}