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
  : Base(node_name_, options_, 1)
  , throttler_(nullptr)
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
  string model_name;
  uint16_t portData;
  get_parameter_or("model", model_name, string("World"));
  get_parameter_or("bridge.Clock", portData, uint16_t(3));

  hashKeySub_ = model_name + GetPartsName();
  DBG_SIM_INFO("hash Key sub: %s", hashKeySub_.c_str());

  double publish_rate;
  get_parameter_or("publish_rate", publish_rate, 50.0);
  DBG_SIM_INFO("publish_rate:%f", publish_rate);

  throttler_ = new Throttler(publish_rate);

  auto pBridgeData = GetBridge(0);

  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeySub_ + "Clock");
  }

  // Offer transient local durability on the clock topic so that if publishing is infrequent,
  // late subscribers can receive the previously published message(s).
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock",
                                                           rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
}

void World::Deinitialize()
{
  delete throttler_;
  DisconnectBridges();
}

void World::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  if (!pbBuf_.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  if (pbBuf_.name() == "timeInfo" &&
      pbBuf_.value().type() == msgs::Any::NONE &&
      pbBuf_.children_size() == 2)
  {
    const auto simTime = (pbBuf_.children(0).name() != "simTime") ? msgs::Time() : pbBuf_.children(0).value().time_value();

    const auto realTime = (pbBuf_.children(1).name() != "realTime") ? msgs::Time() : pbBuf_.children(1).value().time_value();

    PublishSimTime(rclcpp::Time(simTime.sec(), simTime.nsec()),
                   rclcpp::Time(realTime.sec(), realTime.nsec()));
  }
}

void World::PublishSimTime(const rclcpp::Time simTime, const rclcpp::Time realTime)
{
  if (throttler_->IsReady(realTime))
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = simTime;
    clock_pub_->publish(clock);
  }
}