/**
 *  @file   main.cpp
 *  @date   2021-01-16
 *  @author Hyunseok Yang
 *  @brief
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <cloisim_ros_base/base.hpp>
#include <cloisim_ros_bringup_param/bringup_param.hpp>
#include <thread>
#include <vector>

using namespace std;

extern vector<shared_ptr<cloisim_ros::Base>> g_rclcpp_node_list;
extern rclcpp::NodeOptions g_default_node_options;
extern bool g_isSingleMode;

void bringup_cloisim_ros(const Json::Value result_map, const string target_model, const string target_parts_type, const string target_parts_name);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  std::thread main_node_thread;

  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_bringup");
  executor.add_node(bringup_param_node);

  g_isSingleMode = bringup_param_node->IsSingleMode();

  const auto targetModel = bringup_param_node->TargetModel();
  const auto targetPartsType = bringup_param_node->TargetPartsType();
  const auto targetPartsName = bringup_param_node->TargetPartsName();
  const auto result_map = bringup_param_node->GetBringUpList();

  if (result_map.empty())
    cout << " >>>>> Failed to get bringup list!! Check CLOiSim status first!!!" << endl;
  else
  {
    g_default_node_options.append_parameter_override("single_mode", bool(g_isSingleMode));

    bringup_cloisim_ros(result_map, targetModel, targetPartsType, targetPartsName);
  }

  auto cnt = 1;
  while (cnt-- > 0)
  {
    main_node_thread = std::thread([&executor]()
                                   {
                                        for (auto it = g_rclcpp_node_list.begin(); it != g_rclcpp_node_list.end(); ++it)
                                        {
                                          rclcpp::sleep_for(10ms);
                                          cout << "node added: " << (*it)->get_name() << endl;
                                          executor.add_node(*it);
                                        } });
  }

  cout << "Spinning MultiThreadedExecutor NumberOfThread=" << executor.get_number_of_threads() << endl;
  executor.spin();

  rclcpp::shutdown();
  main_node_thread.join();

  return 0;
}