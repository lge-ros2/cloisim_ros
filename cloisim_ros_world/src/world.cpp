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
#include <cloisim_msgs/any.pb.h>
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/time.pb.h>

using string = std::string;

namespace cloisim_ros
{
World::World(const rclcpp::NodeOptions & options_, const string node_name)
: Base(node_name, options_)
{
  Start(false);
}

World::World()
: World(rclcpp::NodeOptions(), "cloisim_ros_world") {}

World::~World()
{
  // cout << "~World()" << endl;
  Stop();
}

void World::Initialize()
{
  uint16_t portClock;
  get_parameter_or("bridge.Clock", portClock, uint16_t(0));
  const auto hashKeyClock = GetModelName() + GetPartsName() + "Clock";

  uint16_t portControl;
  get_parameter_or("bridge.Control", portControl, uint16_t(0));
  const auto hashKeyControl = GetModelName() + GetPartsName() + "Control";

  DBG_SIM_INFO("hashKey Clock(%s) Control(%s)", hashKeyClock.c_str(), hashKeyControl.c_str());

  // Offer transient local durability on the clock topic so that if publishing is infrequent,
  // late subscribers can receive the previously published message(s).
  pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

  client_ = create_client<std_srvs::srv::Empty>("/rviz/reset_time");

  auto data_bridge_clock_ptr = CreateBridge();
  if (data_bridge_clock_ptr != nullptr) {
    data_bridge_clock_ptr->Connect(zmq::Bridge::Mode::SUB, portClock, hashKeyClock);
    AddBridgeReceiveWorker(data_bridge_clock_ptr,
        bind(&World::PublishData, this, std::placeholders::_1));
  }

  auto data_bridge_control_ptr = CreateBridge();
  if (data_bridge_control_ptr != nullptr) {
    data_bridge_control_ptr->Connect(zmq::Bridge::Mode::SERVICE, portControl, hashKeyControl);
    AddBridgeServiceWorker(data_bridge_control_ptr,
        bind(&World::ServiceRequest, this, std::placeholders::_1));
  }
}

void World::Deinitialize()
{
  // cout << "World::Deinitialize()" << endl;
  pub_.reset();
}

void World::PublishData(const string & buffer)
{
  if (!pb_buf_.ParseFromString(buffer)) {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.sim_time());

#if 0
  const auto realTime = pb_buf_.real_time();
#endif

  msg_clock_.clock = GetTime();
  pub_->publish(msg_clock_);
}

std::string World::ServiceRequest(const string & buffer)
{
  cloisim::msgs::Param res_param;
  res_param.set_name("result");
  auto pVal = res_param.mutable_value();
  pVal->set_type(cloisim::msgs::Any::STRING);
  pVal->set_string_value("");

  cloisim::msgs::Param req_param;
  if (req_param.ParseFromString(buffer)) {
    if (req_param.name() == "reset_simulation" &&
      req_param.has_value() && req_param.value().bool_value() == true)
    {
      if (client_ != nullptr) {
        if (client_->wait_for_service(std::chrono::seconds(1))) {
          auto request = std::make_shared<std_srvs::srv::Empty::Request>();
          auto future = client_->async_send_request(request);

          if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            auto result = future.get();
            DBG_SIM_INFO("Reset rviz");
            pVal->set_string_value("OK");
          } else {
            DBG_SIM_ERR("Service call timed out after 2 seconds.");
            pVal->set_string_value("TIMEOUT");
          }
        } else {
          DBG_SIM_ERR("Service '%s' not available after waiting.", client_->get_service_name());
          pVal->set_string_value("SERVICE_UNAVAILABLE");
        }
      }
    }
  } else {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    pVal->set_string_value("SERVICE_DATA_ERROR");
  }

  string response_message;
  res_param.SerializeToString(&response_message);
  return response_message;
}
}  // namespace cloisim_ros
