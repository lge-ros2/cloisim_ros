/**
 *  @file   bringup_param.cpp
 *  @date   2024-06-24
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

#include "cloisim_ros_bringup_param/bringup_param.hpp"

using std::cout;
using std::endl;

namespace cloisim_ros
{

bool BringUpParam::IsRobotSpecificType(const string node_type)
{
  return
    !node_type.compare("MICOM") || !node_type.compare("JOINTCONTROL") ||
    !node_type.compare("LIDAR") || !node_type.compare("LASER") || !node_type.compare("CAMERA") ||
    !node_type.compare("DEPTHCAMERA") || !node_type.compare("MULTICAMERA") ||
    !node_type.compare("SEGMENTCAMERA") || !node_type.compare("REALSENSE") ||
    !node_type.compare("GPS") || !node_type.compare("IMU") || !node_type.compare("SONAR");
}

bool BringUpParam::IsWorldSpecificType(const string node_type)
{
  return
    !node_type.compare("ELEVATOR") || !node_type.compare("WORLD") || !node_type.compare("ACTOR") ||
    !node_type.compare("GROUNDTRUTH");
}

BringUpParam::BringUpParam(const string basename)
: Node(
    basename + "_param", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .allow_undeclared_parameters(false))
  , is_single_mode(false)
  , target_model("")
  , target_parts_type("")
  , target_parts_name("")
  , enable_tf_micom_(true)
  , ws_service_ptr_(nullptr)
{
  get_parameter("single_mode", is_single_mode);
  get_parameter("target_model", target_model);
  get_parameter("target_parts_type", target_parts_type);
  get_parameter("target_parts_name", target_parts_name);

  if (HasEnableTFforMicom()) {
    get_parameter("micom.enable_tf", enable_tf_micom_);
  }

  RCLCPP_INFO_ONCE(
    this->get_logger(), "Params > single_mode(%d) target[model(%s) parts[type(%s) name(%s)]]",
    is_single_mode, target_model.c_str(), target_parts_type.c_str(), target_parts_name.c_str());

  RCLCPP_INFO_ONCE(this->get_logger(), "enable_tf > micom(%d)", (int)enable_tf_micom_);
}

BringUpParam::~BringUpParam()
{
  // cout << __FUNCTION__ << " destructor called" << endl;
  if (ws_service_ptr_ != nullptr) {
    delete ws_service_ptr_;
    ws_service_ptr_ = nullptr;
  }
}

Json::Value BringUpParam::RequestBringUpList()
{
  Json::Reader reader;
  Json::Value result = Json::nullValue;

  if (ws_service_ptr_ == nullptr) {ws_service_ptr_ = new WebSocketService();}

  // cout << "ws Run" << endl;
  ws_service_ptr_->Run();

  while (true) {
    // cout << "ws request" << endl;
    ws_service_ptr_->Request();

    // cout << "start to load payload" << endl;
    const auto payload = ws_service_ptr_->PopPayload();

    if (payload.empty()) {
      if (ws_service_ptr_->IsConnected() == false) {
        // cout << "Not connected yet" << endl;
        result = Json::nullValue;

        delete ws_service_ptr_;
        ws_service_ptr_ = nullptr;

        break;
      } else {
        cout << "PayLoad is empty" << endl;
        continue;
      }
    } else {
      Json::Value root;
      reader.parse(payload, root, false);

      if (root["result"].size() > 0) {
        result = root["result"];
        SetIndividualParameters(result);
        // cout << "There is node map list: " << result.size() << endl;
        // cout << result << endl;
        break;
      } else {
        result = Json::nullValue;
        // cout << "There is no node map list" << endl;
      }
    }
  }

  return result;
}

void BringUpParam::SetIndividualParameters(Json::Value & result)
{
  if (result.isNull()) {return;}

  // cout << __FUNCTION__<< endl;
  // cout << result << endl;
  for (auto it = result.begin(); it != result.end(); it++) {
    const auto node_namespace = it.key().asString();
    auto item_list = (*it);
    // cout << node_namespace <<  endl;

    for (auto it2 = item_list.begin(); it2 != item_list.end(); ++it2) {
      const auto node_type = it2.key().asString();
      auto node_list = (*it2);

      // cout << "\t" << node_type << ", " << endl;
      if (node_type.compare("MICOM") == 0) {
        for (auto it3 = node_list.begin(); it3 != node_list.end(); ++it3) {
          const auto node_name = it3.key().asString();
          auto node_param_list = (*it3);

          if (HasEnableTFforMicom()) {
            node_param_list["enable_tf"] = EnableTFforMicom();
          }

          // cout << "\t\t node_name: " << node_name << endl;
          // for (auto it4 = node_param_list.begin(); it4 != node_param_list.end(); ++it4)
          //   cout << "\t\t\t" << it4.key().asString() << ": " << (*it4).asString() << endl;

          node_list[node_name] = node_param_list;
        }
      }
      item_list[node_type] = node_list;
    }
    result[node_namespace] = item_list;
  }
  // cout << result << endl;
}

Json::Value BringUpParam::GetFilteredListByParameters(const Json::Value result)
{
  Json::Value root;
  // cout << result << endl;
  for (auto it = result.begin(); it != result.end(); it++) {
    const auto node_namespace = it.key().asString();
    const auto item_list = (*it);
    // cout << node_namespace <<  endl;

    for (auto it2 = item_list.begin(); it2 != item_list.end(); ++it2) {
      const auto node_type = it2.key().asString();
      const auto node_list = (*it2);

      // cout << "\t" << node_type << ", " << endl;
      if (target_parts_type.compare(node_type) == 0) {
        const auto node_name = node_list.begin().key().asString();
        // cout << target_parts_name << ", node_name: " << node_name << endl;

        if (
          target_parts_name.empty() ||
          (!target_parts_name.empty() && target_parts_name.compare(node_name) == 0))
        {
          root[node_namespace] = node_list;
        } else {
          root[node_namespace] = Json::Value();
        }
      }
    }
  }

  return root;
}

Json::Value BringUpParam::GetBringUpList(const bool filterByParameters)
{
  const auto bringup_map_list = RequestBringUpList();
  return (filterByParameters) ? GetFilteredListByParameters(bringup_map_list) : bringup_map_list;
}

void BringUpParam::StoreBridgeInfosAsParameters(
  const Json::Value item, rclcpp::NodeOptions & node_options)
{
  if (item.isNull()) {return;}

  for (auto it = item.begin(); it != item.end(); ++it) {
    const auto bridge_key = it.key().asString();
    const auto bridge_port = (*it).asUInt();
    node_options.append_parameter_override("bridge." + bridge_key, uint16_t(bridge_port));
    // cout << "bridge." << bridge_key << " = " << bridge_port << endl;
  }
}

void BringUpParam::StoreFilteredInfoAsParameters(
  const Json::Value item, rclcpp::NodeOptions & node_options)
{
  // cout << item << endl;
  // cout << "model_name: " << model_name << ", " << TargetModel() << endl;
  if (TargetModel().empty()) {TargetModel(item.begin().key().asString());}

  const auto model_name = TargetModel();
  const auto isResultEmpty = item[model_name].empty();
  const auto first_iteration =
    (isResultEmpty) ? Json::ValueConstIterator() : item[model_name].begin();

  if (!isResultEmpty) {TargetPartsName(first_iteration.key().asString());}

  const auto bridge_infos = (isResultEmpty) ? Json::Value() : *first_iteration;
  // cout << bridge_infos << endl;

  node_options.append_parameter_override("single_mode", IsSingleMode());

  if (IsSingleMode()) {node_options.append_parameter_override("single_mode.robotname", model_name);}

  node_options.append_parameter_override("model", TargetModel());

  cloisim_ros::BringUpParam::StoreBridgeInfosAsParameters(bridge_infos, node_options);
}

}  // namespace cloisim_ros
