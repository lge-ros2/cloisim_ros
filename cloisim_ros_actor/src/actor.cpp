/**
 *  @file   Actor.cpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 package for elevator system
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_actor/actor.hpp"

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;
using namespace placeholders;

Actor::Actor(const rclcpp::NodeOptions &options_, const string node_name)
    : Base(node_name, options_)
    , control_bridge_ptr(nullptr)
{
  Start(false);
}

Actor::Actor()
    : Actor(rclcpp::NodeOptions(), "cloisim_ros_actor")
{
}

Actor::~Actor()
{
  Stop();
}

void Actor::Initialize()
{
  const auto nodeName = GetPartsName();
  const auto hashKeySrv = GetModelName() + nodeName + "Control";
  DBG_SIM_INFO("hash Key srv: %s", hashKeySrv.c_str());

  uint16_t portControl;
  get_parameter_or("bridge.Control", portControl, uint16_t(0));

  control_bridge_ptr = CreateBridge(hashKeySrv);
  if (control_bridge_ptr != nullptr)
  {
    control_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portControl, hashKeySrv);

    srvCallMoveActor_ = this->create_service<cloisim_ros_msgs::srv::MoveActor>(nodeName + "/move_actor", bind(&Actor::CallMoveActor, this, _1, _2, _3));
  }
}

void Actor::CallMoveActor(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<cloisim_ros_msgs::srv::MoveActor::Request> request,
    const shared_ptr<cloisim_ros_msgs::srv::MoveActor::Response> response)
{
  const auto message = CreateMoveRequest(request->target_name, request->destination);
  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

bool Actor::GetResultFromResponse(const msgs::Param &response_msg)
{
  if (!response_msg.IsInitialized() || response_msg.name().compare("result") != 0)
  {
    return false;
  }
  return response_msg.value().bool_value();
}

cloisim::msgs::Param Actor::CreateMoveRequest(const string target_name, const geometry_msgs::msg::Vector3 point)
{
  msgs::Param request_msg;
  request_msg.set_name(target_name);

  auto value_ptr = request_msg.mutable_value();
  value_ptr->set_type(msgs::Any::VECTOR3D);

  auto vector3d_value_ptr = value_ptr->mutable_vector3d_value();
  vector3d_value_ptr->set_x(point.x);
  vector3d_value_ptr->set_y(point.y);
  vector3d_value_ptr->set_z(point.z);

  return request_msg;
}