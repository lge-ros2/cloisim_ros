/**
 *  @file   main.cpp
 *  @date   2021-01-16
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
#include <atomic>
#include <cloisim_ros_bringup_param/bringup_param.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "cloisim_ros_bringup/type.hpp"

using namespace std::literals::chrono_literals;
using string = std::string;

extern node_map_t g_node_map_list;

void make_bringup_list(
  const Json::Value & result_map, const string target_model, const string target_parts_type,
  const string target_parts_name, const bool is_single_mode);

void remove_all_bringup_nodes(rclcpp::Executor & executor, const rclcpp::Logger & logger)
{
  ERR(logger, "Remove all nodes(" << g_node_map_list.size() << ")");
  for (auto it = g_node_map_list.cbegin(); it != g_node_map_list.cend(); it++) {
    const auto & key = it->first;
    const auto & value = it->second;

    const auto node = std::get<2>(value);
    executor.remove_node(node);
    const auto node_info = std::get<0>(key) + "/" + std::get<1>(key) + "/" + std::get<2>(key);
    WARN(logger, "> Node(" << node_info << ") removed");
  }

  g_node_map_list.clear();
  // ERR(logger, "Remove all nodes finished");
}

void add_all_bringup_nodes(rclcpp::Executor & executor, const rclcpp::Logger & logger)
{
  (void)logger;

  for (auto it = g_node_map_list.begin(); it != g_node_map_list.end(); ) {
    const auto & key = it->first;
    auto & value = it->second;

    const auto node_info = std::get<0>(key) + "/" + std::get<1>(key) + "/" + std::get<2>(key);
    const auto node = std::get<2>(value);

    if (std::get<0>(value) == false) {
      WARN(logger, "New Node added(" << node_info << ")");
      executor.add_node(node);
      std::get<0>(value) = true;  // set to true for marking added_node
      ++it;
    } else if (std::get<1>(value) == true) {
      executor.remove_node(node);
      g_node_map_list.erase(it++);
      // INFO(logger, "Node removed: " << node_info);
    } else {
      // INFO(logger, "Already node added: " << node_info);
      ++it;
    }
  }
}

void bringup_process(
  std::shared_ptr<cloisim_ros::BringUpParam> param_node, rclcpp::Executor & executor,
  const rclcpp::Logger & logger)
{
  const auto bringup_list_map = param_node->GetBringUpList();
  if (bringup_list_map.empty()) {
    INFO(logger, ">> Check if CLOiSim is launched first!!!");

    if (g_node_map_list.size() > 0) {
      remove_all_bringup_nodes(executor, logger);
    }
  } else {
    const auto is_single_mode = param_node->IsSingleMode();
    const auto targetModel = param_node->TargetModel();
    const auto targetPartsType = param_node->TargetPartsType();
    const auto targetPartsName = param_node->TargetPartsName();

    make_bringup_list(
      bringup_list_map, targetModel, targetPartsType, targetPartsName, is_single_mode);

    add_all_bringup_nodes(executor, logger);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  const auto param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_bringup");
  const auto logger = param_node->get_logger();

  executor.add_node(param_node);

  std::atomic<bool> running_thread{true};

  std::mutex mtx;
  std::condition_variable cv;

  auto interruptible_sleep = [&](auto duration) {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait_for(lk, duration, [&] {return !running_thread.load();});
    };

  rclcpp::on_shutdown([&] {
      running_thread.store(false, std::memory_order_relaxed);
      cv.notify_all();
      executor.cancel();
  });

  auto thread = std::make_unique<std::thread>([&]() {
        static const auto maxRetryNum = 5;
        static const auto waitingTime = 3s;
        static const auto periodicCheckTime = 5s;
        static const auto emptyWaitTime = 500ms;

        auto retry_count = maxRetryNum;

        while (running_thread.load()) {
          bringup_process(param_node, executor, logger);

          if (!running_thread.load()) {break;}

          if (g_node_map_list.size() == 0) {
            interruptible_sleep(emptyWaitTime);

            if (!running_thread.load()) {break;}

            if (retry_count-- > 0) {
              INFO(logger, "Failed to connect to the CLOiSim. Wait " << waitingTime.count() <<
                "sec and retry to connect.");
              INFO(logger, "Remained retrial=" << retry_count);
              interruptible_sleep(waitingTime);
            } else {
              ERR_ONCE(logger, "Finally, failed to connect CLOiSim.");
              rclcpp::shutdown();
              break;
            }
          } else {
            retry_count = maxRetryNum;
            interruptible_sleep(periodicCheckTime);
          }
        }
  });

  WARN_ONCE(logger,
    "Spinning MultiThreadedExecutor NumOfThread=" << executor.get_number_of_threads());

  executor.spin();
  running_thread.store(false);
  cv.notify_all();

  if (thread && thread->joinable()) {
    thread->join();
  }

  rclcpp::shutdown();
  return 0;
}
