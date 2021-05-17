/**
 *  @file   cloisim_ros_world.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 node for controlling unity simulation
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_world/world.hpp"
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/any.pb.h>
#include <cloisim_msgs/time.pb.h>

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;

World::World(const rclcpp::NodeOptions &options_, const std::string node_name_)
  : Base(node_name_, options_)
{
  Start();
}

World::World()
  : World(rclcpp::NodeOptions(), "cloisim_ros_world")
{
}

World::~World()
{
  Stop();
}

void World::Initialize()
{
  uint16_t portClock;
  get_parameter_or("bridge.Clock", portClock, uint16_t(0));

  const auto hashKey = GetModelName() + GetPartsName() +  "Clock";
  DBG_SIM_INFO("hash Key: %s", hashKey.c_str());

  // Offer transient local durability on the clock topic so that if publishing is infrequent,
  // late subscribers can receive the previously published message(s).
  pub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

  auto pBridgeData = CreateBridge(hashKey);
  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portClock, hashKey);
    CreatePublisherThread(pBridgeData);
  }
}

void World::UpdatePublishingData(const string &buffer)
{
  if (!pbBuf.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetSimTime(pbBuf.sim_time());
  // const auto realTime = pbBuf.real_time();

  msgClock.clock = GetSimTime();
  pub->publish(msgClock);
}