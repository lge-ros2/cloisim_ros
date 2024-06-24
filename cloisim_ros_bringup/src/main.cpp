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

#include <thread>

#include <cloisim_ros_base/base.hpp>
#include <cloisim_ros_bringup_param/bringup_param.hpp>

using namespace std;

extern map<tuple<string, string, string>, tuple<bool, bool, shared_ptr<cloisim_ros::Base>>> g_node_map_list;

void make_bringup_list(const Json::Value result_map, const string target_model, const string target_parts_type, const string target_parts_name, const bool is_single_mode);

void remove_all_bringup_nodes(rclcpp::Executor& executor, const rclcpp::Logger& logger)
{
  ERR(logger, "Remove all nodes(" << g_node_map_list.size() << ")");
  for (auto it = g_node_map_list.cbegin(); it != g_node_map_list.cend(); it++)
  {
    const auto& key = it->first;
    const auto& value = it->second;

    const auto node = get<2>(value);
    executor.remove_node(node);
    const auto node_info = get<0>(key) + "/" + get<1>(key) + "/" + get<2>(key);
    WARN(logger, "> Node(" << node_info << ") removed");
  }

  g_node_map_list.clear();
  // ERR(logger, "Remove all nodes finished");
}

void add_all_bringup_nodes(rclcpp::Executor& executor, const rclcpp::Logger& logger)
{
  (void)logger;

  for (auto it = g_node_map_list.begin(); it != g_node_map_list.end();)
  {
    const auto& key = it->first;
    auto& value = it->second;

    const auto node_info = get<0>(key) + "/" + get<1>(key) + "/" + get<2>(key);
    const auto node = get<2>(value);
    if (get<0>(value) == false)
    {
      WARN(logger, "New Node added(" << node_info << ")");
      executor.add_node(node);
      get<0>(value) = true;  // set to true for marking added_node
      ++it;
    }
    else if (get<1>(value) == true)
    {
      executor.remove_node(node);
      g_node_map_list.erase(it++);
      // INFO(logger, "Node removed: " << node_info);
    }
    else
    {
      // INFO(logger, "Already node added: " << node_info);
      ++it;
    }
  }
}

void bringup_process(std::shared_ptr<cloisim_ros::BringUpParam> param_node, rclcpp::Executor& executor, const rclcpp::Logger& logger)
{
  const auto bringup_list_map = param_node->GetBringUpList();
  if (bringup_list_map.empty())
  {
    INFO(logger, ">> Check if CLOiSim is launched first!!!");

    if (g_node_map_list.size() > 0)
      remove_all_bringup_nodes(executor, logger);

    rclcpp::sleep_for(500ms);
  }
  else
  {
    const auto is_single_mode = param_node->IsSingleMode();
    const auto targetModel = param_node->TargetModel();
    const auto targetPartsType = param_node->TargetPartsType();
    const auto targetPartsName = param_node->TargetPartsName();
    // cout << targetModel << ":" << targetPartsType << ":" << targetPartsName << endl;

    make_bringup_list(bringup_list_map, targetModel, targetPartsType, targetPartsName, is_single_mode);

    add_all_bringup_nodes(executor, logger);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  const auto param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_bringup");
  const auto logger = param_node->get_logger();

  executor.add_node(param_node);

  auto running_thread = true;

  auto thread = std::make_unique<std::thread>(
      [&]()
      {
        static const int maxRetryNum = 5;
        static const auto waitingTime = 3s;
        static const auto periodicCheckTime = 2s;

        auto retry_count = maxRetryNum;
        while (retry_count-- > 0 && running_thread)
        {
          bringup_process(param_node, executor, logger);

          // INFO(logger, "bringup_process() size=" << g_node_map_list.size());
          if (g_node_map_list.size() == 0)
          {
            INFO(logger, "Failed to connect to the CLOiSim. "
                             << "Wait " << to_string(waitingTime.count()) << "sec and retry to connnect. "
                             << "Remained retrial=" << retry_count);
            this_thread::sleep_for(waitingTime);
          }
          else
          {
            retry_count = maxRetryNum;
            this_thread::sleep_for(periodicCheckTime);
          }
        }

        ERR_ONCE(logger, "Finally, failed to connect CLOiSim.");
        kill(getpid(), SIGINT);
      });

  WARN_ONCE(logger, "Spinning MultiThreadedExecutor NumOfThread=" << executor.get_number_of_threads());
  executor.spin();

  running_thread = false;

  if (thread->joinable())
    thread->join();

  rclcpp::shutdown();

  return 0;
}
