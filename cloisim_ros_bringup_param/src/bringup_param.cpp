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

#include "cloisim_ros_bringup_param/bringup_param.hpp"

using namespace cloisim_ros;

BringUpParam::BringUpParam(const std::string basename)
    : Node(basename + "_param",
           rclcpp::NodeOptions()
               .automatically_declare_parameters_from_overrides(true)
               .allow_undeclared_parameters(false))
    , isSingleMode(false)
    , target_model("")
    , target_parts_type("")
    , target_parts_name("")
{
  get_parameter("single_mode", isSingleMode);
  get_parameter("target_model", target_model);
  get_parameter("target_parts_type", target_parts_type);
  get_parameter("target_parts_name", target_parts_name);

  RCLCPP_INFO_ONCE(this->get_logger(),
                   "single mode: %d target model: %s target_parts_type: %s target_parts_name: %s",
                   isSingleMode, target_model.c_str(), target_parts_type.c_str(), target_parts_name.c_str());

  wsService = new WebSocketService();
}

Json::Value BringUpParam::GetFilteredListByParameters(const Json::Value result)
{
  Json::Value root;
  // cout << result_map << endl;
  for (auto it = result.begin(); it != result.end(); it++)
  {
    const auto node_namespace = it.key().asString();
    const auto item_list = (*it);
    // cout << node_namespace <<  endl;

    for (auto it2 = item_list.begin(); it2 != item_list.end(); ++it2)
    {
      const auto node_type = it2.key().asString();
      const auto node_list = (*it2);

      // cout << "\t" << node_type << ", " << endl;
      if (target_parts_type.compare(node_type) == 0)
      {
        const auto node_name = node_list.begin().key().asString();
        // cout << target_parts_name << ", node_name: " << node_name << endl;

        if (target_parts_name.empty() || (!target_parts_name.empty() && target_parts_name.compare(node_name) == 0))
          root[node_namespace] = node_list;
        else
          root[node_namespace] = Json::Value();

        return root;
      }
    }
  }

  return root;
}

Json::Value BringUpParam::GetBringUpList(const bool filterByParameters)
{
  Json::Reader reader;
  Json::Value root;

  auto count = maxRetryNum;
  while (count-- > 0)
  {
    // wsService->SetTargetModel(target_model);
    const auto payload = wsService->Run();
    reader.parse(payload, root, false);

    const auto result_map = root["result"];
    if (result_map.size() > 1)
    {
      if (filterByParameters)
      {
        return GetFilteredListByParameters(result_map);
      }
      else
      {
        return result_map;
      }
    }

    cout << "Failed to get all target information " << endl
         << "Wait " << waitingSeconds << " sec and retry to get object info.... " << endl
         << "remained retrial count: " << count << endl;

    sleep(waitingSeconds);
  }

  return root;
}

void BringUpParam::StoreBridgeInfosAsParameters(const Json::Value item, rclcpp::NodeOptions &node_options)
{
  if (!item.isNull() )
  {
    for (auto paramIt = item.begin(); paramIt != item.end(); ++paramIt)
    {
      const auto bridge_key = paramIt.key().asString();
      const auto bridge_port = (*paramIt).asUInt();
      node_options.append_parameter_override("bridge." + bridge_key, uint16_t(bridge_port));
      // cout << "bridge." << bridge_key << " = " << bridge_port << endl;
    }
  }
}