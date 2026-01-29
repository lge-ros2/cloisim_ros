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

#include <algorithm>
#include <sstream>

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

std::string World::ServiceRequest(const std::string & buffer)
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
      const auto services = FindResetTimeServices();
      if (services.empty()) {
        DBG_SIM_ERR("No RViz reset_time service found");
        pVal->set_string_value("SERVICE_UNAVAILABLE");
      } else {
        int ok_count = 0;
        int timeout_count = 0;
        int unavailable_count = 0;

        for (const auto & srv_name : services) {
          auto c = GetOrCreateResetClient(srv_name);
          if (c == nullptr) {
            unavailable_count++;
            continue;
          }

          if (!c->wait_for_service(std::chrono::seconds(1))) {
            unavailable_count++;
            DBG_SIM_WRN("Service '%s' not available.", srv_name.c_str());
            continue;
          }

          auto request = std::make_shared<std_srvs::srv::Empty::Request>();
          auto future = c->async_send_request(request);

          if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            (void)future.get();
            ok_count++;
            DBG_SIM_INFO("Reset RViz via service: %s", srv_name.c_str());
          } else {
            timeout_count++;
            DBG_SIM_WRN("Service call timed out: %s", srv_name.c_str());
          }
        }

        if (ok_count > 0) {
          std::ostringstream oss;
          oss << "OK(" << ok_count << "/" << services.size() << ") timeout=" << timeout_count
              << " unavailable=" << unavailable_count;
          pVal->set_string_value(oss.str());
        } else if (timeout_count > 0) {
          pVal->set_string_value("TIMEOUT");
        } else {
          pVal->set_string_value("SERVICE_UNAVAILABLE");
        }
      }
    }
  } else {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    pVal->set_string_value("SERVICE_DATA_ERROR");
  }

  std::string response_message;
  res_param.SerializeToString(&response_message);
  return response_message;
}

std::vector<std::string> cloisim_ros::World::FindResetTimeServices() const
{
  const auto ends_with = [](const std::string & s, const std::string & suffix) {
      return s.size() >= suffix.size() &&
             s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

  std::vector<std::string> found;
  static const std::string suffix = "/rviz/reset_time";

  const auto services = this->get_service_names_and_types();
  for (const auto & kv : services) {
    const auto & name = kv.first;
    const auto & types = kv.second;

    if (!ends_with(name, suffix)) {
      continue;
    }

    const auto is_empty =
      std::find(types.begin(), types.end(), "std_srvs/srv/Empty") != types.end();

    if (is_empty) {
      found.push_back(name);
    }
  }

  std::sort(found.begin(), found.end());
  return found;
}

rclcpp::Client<std_srvs::srv::Empty>::SharedPtr World::GetOrCreateResetClient(
  const std::string & service_name)
{
  std::lock_guard<std::mutex> lk(rviz_client_mtx_);

  auto it = rviz_clients_.find(service_name);
  if (it != rviz_clients_.end()) {
    return it->second;
  }

  auto c = this->create_client<std_srvs::srv::Empty>(service_name);
  rviz_clients_.emplace(service_name, c);
  return c;
}
}  // namespace cloisim_ros
