/**
 *  @file   driver_sim.cpp
 *  @date   2020-05-22
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 driver Sim base base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <unistd.h>
#include "driver_sim/driver_sim.hpp"

using namespace std;

DriverSim::DriverSim(const string node_name, const int number_of_simbridge)
    : Node(node_name,
           rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)),
      m_bRunThread(false),
      m_node_handle(shared_ptr<rclcpp::Node>(this, [](auto) {}))
{
  string sim_ip;
  int sim_manager_port;
  get_parameter_or("sim.manager_ip", sim_ip, string(""));
  get_parameter_or("sim.manager_port", sim_manager_port, int(0));
  get_parameter_or("sim.model", m_robot_name, string("cloi"));
  get_parameter_or("sim.parts", m_parts_name, string("_parts_"));

  DBG_SIM_INFO("[CONFIG] sim.manage_ip = %s, sim.manage_port = %d", sim_ip.c_str(), sim_manager_port);
  DBG_SIM_INFO("[CONFIG] sim.model = %s", m_robot_name.c_str());
  DBG_SIM_INFO("[CONFIG] sim.part = %s", m_parts_name.c_str());

  m_simBridgeList.reserve(number_of_simbridge);

  for (auto index = 0; index < number_of_simbridge; index++)
  {
    auto pSimBridge = new SimBridge();
    pSimBridge->SetSimMasterAddress(sim_ip);
    pSimBridge->SetPortManagerPort(sim_manager_port);
    m_simBridgeList.push_back(pSimBridge);
  }

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node_handle);
  m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node_handle);
}

DriverSim::~DriverSim()
{
  DBG_SIM_INFO("Delete");

  for (auto pSimBridge : m_simBridgeList)
  {
    delete pSimBridge;
  }
}

void DriverSim::Start()
{
  Initialize();

  m_bRunThread = true;
  m_thread = thread([=]() {
    while (IsRunThread()) {
      UpdateData();
    }});

  auto callback_pub = [this]() -> void {
    PublishStaticTF();
  };

  // ROS2 timer for Publisher
  m_timer = this->create_wall_timer(0.5s, callback_pub);
}

void DriverSim::Stop()
{
  m_bRunThread = false;

  usleep(200);

  if (m_thread.joinable())
  {
    m_thread.join();
    DBG_SIM_INFO("Thread finished");
  }

  Deinitialize();
}

void DriverSim::PublishTF()
{
  m_tf_broadcaster->sendTransform(m_tf_list);
  m_tf_list.clear();
}

void DriverSim::PublishStaticTF()
{
  // Update timestamp
  for (auto _tf : m_static_tf_list)
  {
    _tf.header.stamp = m_simTime;
  }

  m_static_tf_broadcaster->sendTransform(m_static_tf_list);
}

SimBridge* DriverSim::GetSimBridge(const int bridge_index)
{
  return m_simBridgeList.at(bridge_index);
}