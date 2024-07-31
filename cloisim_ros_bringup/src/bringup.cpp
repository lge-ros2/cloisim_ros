/**
 *  @file   bringup.cpp
 *  @date   2023-04-10
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

#include "cloisim_ros_bringup/type.hpp"
#include <cloisim_ros_actor/actor.hpp>
#include <cloisim_ros_bringup_param/bringup_param.hpp>
#include <cloisim_ros_camera/camera.hpp>
#include <cloisim_ros_camera/depth_camera.hpp>
#include <cloisim_ros_camera/segmentation_camera.hpp>
#include <cloisim_ros_elevator_system/elevator_system.hpp>
#include <cloisim_ros_gps/gps.hpp>
#include <cloisim_ros_ground_truth/ground_truth.hpp>
#include <cloisim_ros_imu/imu.hpp>
#include <cloisim_ros_joint_control/joint_control.hpp>
#include <cloisim_ros_lidar/lidar.hpp>
#include <cloisim_ros_micom/micom.hpp>
#include <cloisim_ros_multicamera/multicamera.hpp>
#include <cloisim_ros_realsense/realsense.hpp>
#include <cloisim_ros_sonar/sonar.hpp>
#include <cloisim_ros_world/world.hpp>

using namespace std::literals::chrono_literals;
using string = std::string;
using std::cout;
using std::endl;

// <model_name, node_type, node_name>, <is_added, do_remove, shared_ptr>
node_map_t g_node_map_list;

// <model_name, node_type, node_name>
static loaded_key_list_t loaded_key_list;
static bool g_enable_single_mode = false;

static std::shared_ptr<cloisim_ros::Base> make_device_node(
  rclcpp::NodeOptions & node_options, const string & node_type, const string & model_name,
  const string & node_name, const Json::Value & node_param)
{
  std::shared_ptr<cloisim_ros::Base> node = nullptr;

  if (g_enable_single_mode) {
    node_options.append_parameter_override("single_mode.robotname", model_name);
  }

  if (!node_type.compare("MICOM")) {
    if (!node_param["enable_tf"].isNull()) {
      node_options.append_parameter_override("enable_tf", node_param["enable_tf"].asBool());
    }

    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::Micom>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Micom>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("JOINTCONTROL")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::JointControl>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::JointControl>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("LIDAR") || !node_type.compare("LASER")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("CAMERA")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::Camera>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Camera>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("DEPTHCAMERA")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("MULTICAMERA")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("REALSENSE")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::RealSense>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::RealSense>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("SEGMENTCAMERA")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::SegmentationCamera>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::SegmentationCamera>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("GPS")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::Gps>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Gps>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("IMU")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::Imu>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Imu>(node_options, node_name, model_name);
    }
  } else if (!node_type.compare("SONAR")) {
    if (g_enable_single_mode) {
      node = std::make_shared<cloisim_ros::Sonar>(node_options, node_name);
    } else {
      node = std::make_shared<cloisim_ros::Sonar>(node_options, node_name, model_name);
    }
  } else {
  }

  return node;
}

static std::shared_ptr<cloisim_ros::Base> make_world_node(
  rclcpp::NodeOptions & node_options, const string & node_type, const string & model_name,
  const string & node_name)
{
  std::shared_ptr<cloisim_ros::Base> node = nullptr;

  node_options.append_parameter_override("model", model_name);

  if (!node_type.compare("ELEVATOR")) {
    node = std::make_shared<cloisim_ros::ElevatorSystem>(node_options, node_name);
  } else if (!node_type.compare("WORLD")) {
    node = std::make_shared<cloisim_ros::World>(node_options, node_name);
  } else if (!node_type.compare("GROUNDTRUTH")) {
    node = std::make_shared<cloisim_ros::GroundTruth>(node_options, node_name);
  } else if (!node_type.compare("ACTOR")) {
    node = std::make_shared<cloisim_ros::Actor>(node_options, node_name);
  }

  return node;
}

static void parse_target_parts_by_name(
  const Json::Value & item, const string node_type, const string model_name, const string node_name)
{
  std::shared_ptr<cloisim_ros::Base> node = nullptr;

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("single_mode", static_cast<bool>(g_enable_single_mode));
  cloisim_ros::BringUpParam::StoreBridgeInfosAsParameters(item, node_options);

  const auto key = tie(model_name, node_type, node_name);
  loaded_key_list.push_back(key);

  // const auto node_info = "Target Model/Type/Parts: " +
  //                        model_name + "/" + node_type + "/" + node_name;
  // cout << "node info: " << node_info << endl;
  if (g_node_map_list.find(key) != g_node_map_list.end()) {
    // cout << "Already node added - " << node_info << endl;
    return;
  }

  if (cloisim_ros::BringUpParam::IsRobotSpecificType(node_type)) {
    node = make_device_node(node_options, node_type, model_name, node_name, item);
  } else if (cloisim_ros::BringUpParam::IsWorldSpecificType(node_type)) {
    node = make_world_node(node_options, node_type, model_name, node_name);
  } else {
    cout << node_type << " is NOT supported." << endl;
  }

  if (node != nullptr) {
    // cout << "New node added - " << node_info << endl;
    g_node_map_list.insert({key, make_tuple(false, false, node)});
  }
}

static void parse_target_parts_by_type(
  const Json::Value & node_list, const string node_type, const string model_name,
  const string targetPartsName)
{
  // cout << "\tNode Type(Target Parts Type): " << node_type << endl;

  for (auto it3 = node_list.begin(); it3 != node_list.end(); ++it3) {
    const auto node_name = it3.key().asString();
    const auto item = (*it3);

    if (!targetPartsName.empty() && targetPartsName.compare(node_name) != 0) {
      continue;
    } else {
      parse_target_parts_by_name(item, node_type, model_name, node_name);
    }

    // cout << "\t\tNode Name(Target Parts Name): " << node_name << endl;
    // cout << item << endl;
  }
}

static void parse_target_model(
  const Json::Value & item_list, const string item_name, const string targetPartsType,
  const string targetPartsName)
{
  // cout << "Item Name(Target Model): " << item_name << endl;

  for (auto it2 = item_list.begin(); it2 != item_list.end(); ++it2) {
    const auto node_type = it2.key().asString();
    const auto node_list = (*it2);

    if (!targetPartsType.empty() && targetPartsType.compare(node_type) != 0) {
      continue;
    } else {
      parse_target_parts_by_type(node_list, node_type, item_name, targetPartsName);
    }
  }
}

void make_bringup_list(
  const Json::Value & result_map, const string target_model, const string target_parts_type,
  const string target_parts_name, const bool is_single_mode)
{
  g_enable_single_mode = is_single_mode;
  loaded_key_list.clear();

  for (auto it = result_map.begin(); it != result_map.end(); it++) {
    const auto item_name = it.key().asString();
    const auto item_list = (*it);

    if (!target_model.empty() && target_model.compare(item_name) != 0) {
      continue;
    } else {
      parse_target_model(item_list, item_name, target_parts_type, target_parts_name);
      std::this_thread::sleep_for(100ms);
    }
  }

  // for (auto& it : loaded_key_list)
  // {
  //   const auto node_info = get<0>(it) + "/" + get<1>(it) + "/" + get<2>(it);
  //   cout << "Loaded key list=" << node_info << endl;
  // }

  // mark node to remove
  for (auto it = g_node_map_list.begin(); it != g_node_map_list.end(); ++it) {
    const auto key = it->first;
    // const auto node_info = get<0>(key) + "/" + get<1>(key) + "/" + get<2>(key);
    // cout << node_info << endl;
    if (find(loaded_key_list.begin(), loaded_key_list.end(), key) == loaded_key_list.end()) {
      auto & value = it->second;
      if (std::get<1>(value) == false) {
        std::get<1>(value) = true;                                  //  mark to remove
      }
    }
  }
}
