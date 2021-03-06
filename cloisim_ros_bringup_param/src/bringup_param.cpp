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

bool BringUpParam::IsRobotSpecificType(const string node_type)
{
  return (!node_type.compare("MICOM") || !node_type.compare("JOINTCONTROL") ||
          !node_type.compare("LIDAR") || !node_type.compare("LASER") ||
          !node_type.compare("CAMERA") || !node_type.compare("DEPTHCAMERA") || !node_type.compare("MULTICAMERA") || !node_type.compare("REALSENSE") ||
          !node_type.compare("GPS"));
}

bool BringUpParam::IsWorldSpecificType(const string node_type)
{
  return (!node_type.compare("ELEVATOR") || !node_type.compare("ACTOR") || !node_type.compare("WORLD") || !node_type.compare("GROUNDTRUTH"));
}

BringUpParam::BringUpParam(const string basename)
    : Node(basename + "_param",
           rclcpp::NodeOptions()
               .automatically_declare_parameters_from_overrides(true)
               .allow_undeclared_parameters(false))
    , is_single_mode(false)
    , target_model("")
    , target_parts_type("")
    , target_parts_name("")
{
  get_parameter("single_mode", is_single_mode);
  get_parameter("target_model", target_model);
  get_parameter("target_parts_type", target_parts_type);
  get_parameter("target_parts_name", target_parts_name);

  RCLCPP_INFO_ONCE(this->get_logger(),
                   "BringUpParams => single mode: %d, target model: %s, target_parts_type: %s, target_parts_name: %s",
                   is_single_mode, target_model.c_str(), target_parts_type.c_str(), target_parts_name.c_str());

  ws_service_ptr_ = new WebSocketService();

  RequestBringUpList();
}

void BringUpParam::RequestBringUpList()
{
  Json::Reader reader;
  Json::Value root;

  auto count = maxRetryNum;
  while (count-- > 0)
  {
    const auto payload = ws_service_ptr_->Run();
    reader.parse(payload, root, false);

    const auto result_map = root["result"];
    if (result_map.size() > 1)
    {
      result_map_ = result_map;
      break;
    }

    cout << "Failed to get all target information " << endl
         << "Wait " << waitingSeconds << " sec and retry to get object info.... " << endl
         << "remained retrial count: " << count << endl;

    sleep(waitingSeconds);
  }
}

Json::Value BringUpParam::GetFilteredListByParameters(const Json::Value result)
{
  Json::Value root;
  // cout << result << endl;
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
      }
    }
  }

  return root;
}

Json::Value BringUpParam::GetBringUpList(const bool filterByParameters)
{
  return (filterByParameters)? GetFilteredListByParameters(result_map_):result_map_;
}

void BringUpParam::StoreBridgeInfosAsParameters(const Json::Value item, rclcpp::NodeOptions &node_options)
{
  if (!item.isNull() )
  {
    for (auto it = item.begin(); it != item.end(); ++it)
    {
      const auto bridge_key = it.key().asString();
      const auto bridge_port = (*it).asUInt();
      node_options.append_parameter_override("bridge." + bridge_key, uint16_t(bridge_port));
      // cout << "bridge." << bridge_key << " = " << bridge_port << endl;
    }
  }
}

void BringUpParam::StoreFilteredInfoAsParameters(const Json::Value item, rclcpp::NodeOptions &node_options)
{
  // cout << item << endl;
  // const auto model_name = ;
  // cout << "model_name: " << model_name << endl;
  // cout << "model_name: " << TargetModel() << endl;
  if (TargetModel().empty())
    TargetModel(item.begin().key().asString());

  const auto model_name = TargetModel();
  const auto isResultEmpty = item[model_name].empty();
  const auto first_iteration = (isResultEmpty) ? Json::ValueConstIterator() : item[model_name].begin();

  if (!isResultEmpty)
    TargetPartsName(first_iteration.key().asString());

  const auto bridge_infos = (isResultEmpty) ? Json::Value() : *first_iteration;
  // cout << bridge_infos << endl;

  node_options.append_parameter_override("single_mode", IsSingleMode());

  if (IsSingleMode())
    node_options.append_parameter_override("single_mode.robotname", model_name);

  node_options.append_parameter_override("model", TargetModel());

  cloisim_ros::BringUpParam::StoreBridgeInfosAsParameters(bridge_infos, node_options);
}