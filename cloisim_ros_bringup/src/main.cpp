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

#include <cloisim_ros_bringup/bringup_param.hpp>
#include <cloisim_ros_bringup/websocket_service.hpp>

using namespace std;

int main(int argc, char** argv)
{
  const auto env_bridge_ip = getenv("CLOISIM_BRIDGE_IP");
  const auto env_service_port = getenv("CLOISIM_SERVICE_PORT");

  const auto bridge_ip = string((env_bridge_ip == nullptr)? "127.0.0.1": env_bridge_ip);
  const auto service_port = string((env_service_port == nullptr)? "8080":env_service_port);

  Json::Value result_map;

  auto wsService = new cloisim_ros::WebSocketService(bridge_ip, service_port);

  while (true)
  {
    // wsService->SetTarget("cloi1");
    result_map = wsService->Run();

    if (result_map.size() > 1)
    {
      break;
    }

    static const int waitseconds = 3;
    cout << "Failed to get all target information " << endl;
    cout << "Wait " << waitseconds << "sec and retry to get object info.... " << endl;
    sleep(waitseconds);
  }

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  const auto bringup_param_node = std::make_shared<BringUpParam>();
  executor.add_node(bringup_param_node);

  const auto isSingleMode = bringup_param_node->IsSingleMode();

  rclcpp::NodeOptions default_node_options;
  default_node_options.append_parameter_override("singlemode", bool(isSingleMode));

  std::vector<std::shared_ptr<cloisim_ros::Base>> rclcpp_node_list;

  for (auto it = result_map.begin(); it != result_map.end(); it++)
  {
    const auto item_name = it.key().asString();
    const auto item_list = (*it);

    cout << "Item Name: " << item_name << endl;

    for (auto it2 = item_list.begin(); it2 != item_list.end(); ++it2)
    {
      const auto node_type = it2.key().asString();
      const auto node_list = (*it2);

      cout << "\tNode Type: " << node_type << endl;

      for (auto it3 = node_list.begin(); it3 != node_list.end(); ++it3)
      {
        const auto node_name = it3.key().asString();
        const auto item = (*it2)[node_name];

        cout << "\t\tNode Name: " << node_name << endl;
        // cout << item << endl;

        std::shared_ptr<cloisim_ros::Base> node = nullptr;
        rclcpp::NodeOptions node_options(default_node_options);

        if (isSingleMode)
        {
          node_options.append_parameter_override("singlemode.robotname", item_name);
        }

        // store parameters
        for (auto paramIt = item.begin(); paramIt != item.end(); ++paramIt)
        {
          const auto bridge_key = paramIt.key().asString();
          const auto bridge_port = (*paramIt).asUInt();
          node_options.append_parameter_override("bridge." + bridge_key, uint16_t(bridge_port));
          // cout << "bridge." << bridge_key << " = " << bridge_port << endl;
        }

        if (!node_type.compare("MICOM"))
        {
          if (isSingleMode)
          {
            node = std::make_shared<cloisim_ros::Micom>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::Micom>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("LIDAR") || !node_type.compare("LASER"))
        {
          if (isSingleMode)
          {

            node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::Lidar>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("CAMERA"))
        {
          if (isSingleMode)
          {
            node = std::make_shared<cloisim_ros::Camera>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::Camera>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("DEPTHCAMERA"))
        {
          if (isSingleMode)
          {
            node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::DepthCamera>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("MULTICAMERA"))
        {
          if (isSingleMode)
          {
            node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::MultiCamera>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("REALSENSE"))
        {
          if (isSingleMode)
          {
            node = std::make_shared<cloisim_ros::RealSense>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::RealSense>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("GPS"))
        {
          if (isSingleMode)
          {
            node = std::make_shared<cloisim_ros::Gps>(node_options, node_name);
          }
          else
          {
            node = std::make_shared<cloisim_ros::Gps>(node_options, node_name, item_name);
          }
        }
        else if (!node_type.compare("ELEVATOR"))
        {
          node_options.append_parameter_override("model", item_name);
          node = std::make_shared<cloisim_ros::ElevatorSystem>(node_options, node_name);
        }
        else if (!node_type.compare("WORLD"))
        {
          node_options.append_parameter_override("model", item_name);
          node = std::make_shared<cloisim_ros::World>(node_options, node_name);
        }
        else
        {
          cout << node_type << " is not supported." << endl;
          continue;
        }

        rclcpp_node_list.push_back(node);
      }
    }
  }

  for (auto it = rclcpp_node_list.begin(); it != rclcpp_node_list.end(); ++it)
  {
    executor.add_node(*it);
  }

  executor.spin();
  rclcpp::shutdown();

  return 0;
}