/**
 *  @file   bringup.cpp
 *  @date   2023-04-10
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

#include <cloisim_ros_actor/actor.hpp>
#include <cloisim_ros_bringup_param/bringup_param.hpp>
#include <cloisim_ros_camera/camera.hpp>
#include <cloisim_ros_depthcamera/depthcamera.hpp>
#include <cloisim_ros_elevator_system/elevator_system.hpp>
#include <cloisim_ros_gps/gps.hpp>
#include <cloisim_ros_ground_truth/ground_truth.hpp>
#include <cloisim_ros_imu/imu.hpp>
#include <cloisim_ros_joint_control/joint_control.hpp>
#include <cloisim_ros_lidar/lidar.hpp>
#include <cloisim_ros_micom/micom.hpp>
#include <cloisim_ros_multicamera/multicamera.hpp>
#include <cloisim_ros_realsense/realsense.hpp>
#include <cloisim_ros_world/world.hpp>

using namespace std;

typedef int num_of_threads_t;

vector<shared_ptr<cloisim_ros::Base>> g_rclcpp_node_list;
rclcpp::NodeOptions g_default_node_options;
bool g_isSingleMode = false;

void bringup_target_parts_by_name(const Json::Value item, const string node_type, const string model_name, const string node_name)
{
  std::shared_ptr<cloisim_ros::Base> node = nullptr;
  rclcpp::NodeOptions node_options(g_default_node_options);

  cloisim_ros::BringUpParam::StoreBridgeInfosAsParameters(item, node_options);

  cout << model_name << " " << node_type << " " << node_name << endl;

  num_of_threads_t num_of_threads = 1;  // default tf and tf_static for robot specific type

  if (cloisim_ros::BringUpParam::IsRobotSpecificType(node_type))
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
    else if (!node_type.compare("JOINTCONTROL"))
    {
      if (g_isSingleMode)
        node = std::make_shared<cloisim_ros::JointControl>(node_options, node_name);
      else
        node = std::make_shared<cloisim_ros::JointControl>(node_options, node_name, model_name);
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
      num_of_threads += 3;

      if (g_isSingleMode)
        node = std::make_shared<cloisim_ros::Camera>(node_options, node_name);
      else
        node = std::make_shared<cloisim_ros::Camera>(node_options, node_name, model_name);
    }
    else if (!node_type.compare("DEPTHCAMERA"))
    {
      num_of_threads += 3;

      if (g_isSingleMode)
        node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name);
      else
        node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name, model_name);
    }
    else if (!node_type.compare("MULTICAMERA"))
    {
      num_of_threads += 4;

      if (g_isSingleMode)
        node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name);
      else
        node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name, model_name);
    }
    else if (!node_type.compare("REALSENSE"))
    {
      num_of_threads += 9;

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
    else if (!node_type.compare("IMU"))
    {
      if (g_isSingleMode)
        node = std::make_shared<cloisim_ros::Imu>(node_options, node_name);
      else
        node = std::make_shared<cloisim_ros::Imu>(node_options, node_name, model_name);
    }
  }
  else if (cloisim_ros::BringUpParam::IsWorldSpecificType(node_type))
  {
    node_options.append_parameter_override("model", model_name);
    num_of_threads = 1;

    if (!node_type.compare("ELEVATOR"))
    {
      node = std::make_shared<cloisim_ros::ElevatorSystem>(node_options, node_name);
    }
    else if (!node_type.compare("WORLD"))
    {
      node = std::make_shared<cloisim_ros::World>(node_options, node_name);
    }
    else if (!node_type.compare("GROUNDTRUTH"))
    {
      node = std::make_shared<cloisim_ros::GroundTruth>(node_options, node_name);
    }
    else if (!node_type.compare("ACTOR"))
    {
      node = std::make_shared<cloisim_ros::Actor>(node_options, node_name);
    }
  }
  else
  {
    cout << node_type << " is not supported." << endl;
    num_of_threads = 0;
    return;
  }

  if (node != nullptr)
  {
    g_rclcpp_node_list.push_back(node);
  }
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

    usleep(1000);
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