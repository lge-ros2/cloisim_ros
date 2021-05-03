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

#include <cloisim_ros_camera/camera.hpp>
#include <cloisim_ros_multicamera/multicamera.hpp>
#include <cloisim_ros_depthcamera/depthcamera.hpp>
#include <cloisim_ros_realsense/realsense.hpp>
#include <cloisim_ros_gps/gps.hpp>
#include <cloisim_ros_lidar/lidar.hpp>
#include <cloisim_ros_micom/micom.hpp>
#include <cloisim_ros_elevatorsystem/elevatorsystem.hpp>
#include <cloisim_ros_world/world.hpp>
#include <cloisim_ros_bringup_param/bringup_param.hpp>

using namespace std;

static rclcpp::NodeOptions g_default_node_options;
static vector<shared_ptr<cloisim_ros::Base>> g_rclcpp_node_list;
static bool g_isSingleMode = false;

inline static bool isRobotSpecific(const string node_type)
{
  return (!node_type.compare("MICOM") || !node_type.compare("LIDAR") || !node_type.compare("LASER") ||
          !node_type.compare("CAMERA") || !node_type.compare("DEPTHCAMERA") || !node_type.compare("MULTICAMERA") || !node_type.compare("REALSENSE") ||
          !node_type.compare("GPS"));
}

inline static bool isWorldSpecific(const string node_type)
{
  return (!node_type.compare("ELEVATOR") || !node_type.compare("WORLD"));
}

void bringup_target_parts_by_name(const Json::Value item, const string node_type, const string model_name, const string node_name)
{
    std::shared_ptr<cloisim_ros::Base> node = nullptr;
    rclcpp::NodeOptions node_options(g_default_node_options);

    cloisim_ros::BringUpParam::StoreBridgeInfosAsParameters(item, node_options);

    if (isRobotSpecific(node_type))
    {
      if (g_isSingleMode)
        node_options.append_parameter_override("single_mode.robotname", model_name);

      if (!node_type.compare("MICOM"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::Micom>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::Micom>(node_options, node_name, model_name);
      }
      else if (!node_type.compare("LIDAR") || !node_type.compare("LASER"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name, model_name);
      }
      else if (!node_type.compare("CAMERA"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::Camera>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::Camera>(node_options, node_name, model_name);
      }
      else if (!node_type.compare("DEPTHCAMERA"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name, model_name);
      }
      else if (!node_type.compare("MULTICAMERA"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name, model_name);
      }
      else if (!node_type.compare("REALSENSE"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::RealSense>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::RealSense>(node_options, node_name, model_name);
      }
      else if (!node_type.compare("GPS"))
      {
        if (g_isSingleMode)
          node = std::make_shared<cloisim_ros::Gps>(node_options, node_name);
        else
          node = std::make_shared<cloisim_ros::Gps>(node_options, node_name, model_name);
      }
    }
    else if (isWorldSpecific(node_type))
    {
      node_options.append_parameter_override("model", model_name);
      if (!node_type.compare("ELEVATOR"))
      {
        node = std::make_shared<cloisim_ros::ElevatorSystem>(node_options, node_name);
      }
      else if (!node_type.compare("WORLD"))
      {
        node = std::make_shared<cloisim_ros::World>(node_options, node_name);
      }
    }
    else
    {
      cout << node_type << " is not supported." << endl;
      return;
    }

    g_rclcpp_node_list.push_back(node);
}

void bringup_target_parts_by_type(const Json::Value node_list, const string node_type, const string model_name, const string targetPartsName)
{
  cout << "\tNode Type(Target Parts Type): " << node_type << endl;

  for (auto it3 = node_list.begin(); it3 != node_list.end(); ++it3)
  {
    const auto node_name = it3.key().asString();
    const auto item = (*it3);

    if (!targetPartsName.empty() && targetPartsName.compare(node_name) != 0)
    {
      continue;
    }

    cout << "\t\tNode Name(Target Parts Name): " << node_name << endl;
    // cout << item << endl;

    bringup_target_parts_by_name(item, node_type, model_name, node_name);

    usleep(10000);
  }
}

void bringup_target_model(const Json::Value item_list, const string item_name, const string targetPartsType, const string targetPartsName)
{
  cout << "Item Name(Target Model): " << item_name << endl;

  for (auto it2 = item_list.begin(); it2 != item_list.end(); ++it2)
  {
    const auto node_type = it2.key().asString();
    const auto node_list = (*it2);

    if (!targetPartsType.empty() && targetPartsType.compare(node_type) != 0)
    {
      continue;
    }

    bringup_target_parts_by_type(node_list, node_type, item_name, targetPartsName);
   }
}

void bringup_cloisim_ros(const Json::Value result_map, const string target_model, const string target_parts_type, const string target_parts_name)
{
  for (auto it = result_map.begin(); it != result_map.end(); it++)
  {
    const auto item_name = it.key().asString();
    const auto item_list = (*it);

    if (!target_model.empty() && target_model.compare(item_name) != 0)
    {
      continue;
    }

    bringup_target_model(item_list, item_name, target_parts_type, target_parts_name);
  }

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  const auto bringup_param_node = std::make_shared<cloisim_ros::BringUpParam>("cloisim_ros_bringup");
  executor.add_node(bringup_param_node);

  g_isSingleMode = bringup_param_node->IsSingleMode();
  const auto targetModel = bringup_param_node->TargetModel();
  const auto targetPartsType = bringup_param_node->TargetPartsType();
  const auto targetPartsName = bringup_param_node->TargetPartsName();

  g_default_node_options.append_parameter_override("single_mode", bool(g_isSingleMode));

  const auto result_map = bringup_param_node->GetBringUpList();
  if (!result_map.empty())
  {
    bringup_cloisim_ros(result_map, targetModel, targetPartsType, targetPartsName);
    for (auto it = g_rclcpp_node_list.begin(); it != g_rclcpp_node_list.end(); ++it)
    {
      executor.add_node(*it);
      usleep(1000);
    }

    executor.spin();
  }
  else
  {
    cout << " >>>>> Failed to get bringup list!! Check CLOiSim status first!!!" << endl;
  }

  rclcpp::shutdown();

  return 0;
}