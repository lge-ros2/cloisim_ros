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
using namespace placeholders;
using namespace cloisim;
using namespace cloisim_ros;
using namespace elevator_system_msgs;

#define BindCallback(func) bind(&Actor::func, this, _1, _2, _3)

Actor::Actor(const rclcpp::NodeOptions &options_, const string node_name)
    : Base(node_name, options_)
    , srv_mode_(false)
    , control_bridge_ptr(nullptr)
{
  Start(false);
}Actor
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

    const auto reply = RequestReplyMessage(control_bridge_ptr, "request_system_name");
    if (reply.IsInitialized())
    {
      if (reply.name().compare("request_system_name") == 0 &&
          reply.value().type() == msgs::Any_ValueType_STRING &&
          !reply.value().string_value().empty())
      {
        const auto system_name = reply.value().string_value();

        request_msg_.set_name(system_name);
      }
      else
      {
        request_msg_.set_name("Actor");
      }
    }
  }

  srvCallElevator_ = this->create_service<srv::CallElevator>(
      nodeName + string("/call_elevator"), BindCallback(CallElevator));

  srvGetCalledElevator_ = this->create_service<srv::CallElevator>(
      nodeName + string("/get_called_elevator"), BindCallback(GetElevatorCalled));
}

void Actor::CallElevator(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::CallElevator::Request> request,
    const shared_ptr<srv::CallElevator::Response> response)
{
  const auto message = CreateRequest("call_elevator",
                               request->current_floor,
                               request->target_floor);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void Actor::GetElevatorCalled(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::CallElevator::Request> request,
    const shared_ptr<srv::CallElevator::Response> response)
{
  auto message = CreateRequest("get_called_elevator",
                               request->current_floor,
                               request->target_floor);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
      response->result = GetResultFromResponse(reply);

      const auto result_param = reply.children(2);
      if (result_param.IsInitialized())
      {
        const auto elevator_index = (result_param.name().compare("elevator_index") != 0) ? "":result_param.value().string_value();
        response->elevator_index = elevator_index;
      }
  }
}

void Actor::GetElevatorInfo(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::GetElevatorInformation::Request> request,
    const shared_ptr<srv::GetElevatorInformation::Response> response)
{
  auto message = CreateRequest("get_elevator_information", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    msgs::Param result_param;

    response->result = GetResultFromResponse(reply);

    result_param = reply.children(3);
    if (result_param.IsInitialized())
    {
      const auto current_floor = (result_param.name().compare("current_floor") != 0) ? "" : result_param.value().string_value();
      response->current_floor = current_floor;
    }

    result_param = reply.children(4);
    if (result_param.IsInitialized())
    {
      const auto height = (float)((result_param.name().compare("height") != 0) ? 0.0 : result_param.value().double_value());
      response->height = height;
    }
  }
}

void Actor::SelectFloor(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::SelectElevatorFloor::Request> request,
    const shared_ptr<srv::SelectElevatorFloor::Response> response)
{
  auto message = CreateRequest("select_elevator_floor",
                               request->current_floor,
                               request->target_floor,
                               request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void Actor::RequestDoorOpen(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::RequestDoor::Request> request,
    const shared_ptr<srv::RequestDoor::Response> response)
{
  auto message = CreateRequest("request_door_open", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void Actor::RequestDoorClose(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::RequestDoor::Request> request,
    const shared_ptr<srv::RequestDoor::Response> response)
{
  auto message = CreateRequest("request_door_close", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void Actor::IsDoorOpened(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::RequestDoor::Request> request,
    const shared_ptr<srv::RequestDoor::Response> response)
{
  auto message = CreateRequest("is_door_opened", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void Actor::ReserveElevator(
  const shared_ptr<rmw_request_id_t> /*request_header*/,
  const shared_ptr<srv::ReturnBool::Request> /*request*/,
  const shared_ptr<srv::ReturnBool::Response> response)
{
  response->result = srv_mode_;
}

void Actor::ReleaseElevator(
  const shared_ptr<rmw_request_id_t> /*request_header*/,
  const shared_ptr<srv::ReturnBool::Request> /*request*/,
  const shared_ptr<srv::ReturnBool::Response> response)
{
  response->result = srv_mode_;
}

/**
 * @brief Create protobuf message
 *
 * @param service_name
 * @param current_floor
 * @param target_floor
 * @param elevator_index
 *
 * @return created protobuf message for request
 */
msgs::Param Actor::CreateRequest(
    const string service_name,
    const string current_floor,
    const string target_floor,
    const string elevator_index)
{
  request_msg_.clear_children();

  msgs::Param *param_ptr;
  msgs::Any *value_ptr;

  param_ptr = request_msg_.add_children();
  param_ptr->set_name("service_name");
  value_ptr = param_ptr->mutable_value();
  value_ptr->set_type(msgs::Any::STRING);
  value_ptr->set_string_value(service_name);

  param_ptr = request_msg_.add_children();
  param_ptr->set_name("current_floor");
  value_ptr = param_ptr->mutable_value();
  value_ptr->set_type(msgs::Any::STRING);
  value_ptr->set_string_value(current_floor);

  param_ptr = request_msg_.add_children();
  param_ptr->set_name("target_floor");
  value_ptr = param_ptr->mutable_value();
  value_ptr->set_type(msgs::Any::STRING);
  value_ptr->set_string_value(target_floor);

  param_ptr = request_msg_.add_children();
  param_ptr->set_name("elevator_index");
  value_ptr = param_ptr->mutable_value();
  value_ptr->set_type(msgs::Any::STRING);
  value_ptr->set_string_value(elevator_index);

  return request_msg_;
}

bool Actor::GetResultFromResponse(const msgs::Param &response_msg, const int children_index)
{
  const auto result_param = response_msg.children(children_index);
  if (!result_param.IsInitialized() || result_param.name().compare("result") != 0)
  {
    return false;
  }

  return result_param.value().bool_value();
}