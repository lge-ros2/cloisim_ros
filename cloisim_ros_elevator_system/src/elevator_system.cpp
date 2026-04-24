/**
 *  @file   elevator_system.cpp
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

#include "cloisim_ros_elevator_system/elevator_system.hpp"

#include <cloisim_ros_base/param_helper.hpp>

using namespace std::placeholders;
using std::shared_ptr;
using string = std::string;

namespace cloisim_ros
{
#define BindCallback(func) bind(&ElevatorSystem::func, this, _1, _2, _3)

using RequestDoor = elevator_system_msgs::srv::RequestDoor;
using SelectElevatorFloor = elevator_system_msgs::srv::SelectElevatorFloor;
using CallElevator = elevator_system_msgs::srv::CallElevator;
using GetElevatorInformation = elevator_system_msgs::srv::GetElevatorInformation;
using ReturnBool = elevator_system_msgs::srv::ReturnBool;

ElevatorSystem::ElevatorSystem(const rclcpp::NodeOptions & options_, const string node_name)
: Base(node_name, options_), srv_mode_(false), control_bridge_ptr(nullptr)
{
  Start(false);
}

ElevatorSystem::ElevatorSystem()
: ElevatorSystem(rclcpp::NodeOptions(), "cloisim_ros_elevator_system")
{
}

ElevatorSystem::~ElevatorSystem() {Stop();}

void ElevatorSystem::Initialize()
{
  const auto nodeName = GetPartsName();
  const auto hashKeySrv = GetModelName() + nodeName + "Control";
  LOG_I(this, "hashKey: srv(" << hashKeySrv << ")");

  uint16_t portControl;
  get_parameter_or("bridge.Control", portControl, uint16_t(0));

  control_bridge_ptr = CreateBridge();
  if (control_bridge_ptr != nullptr) {
    control_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portControl, hashKeySrv);

    const auto reply = RequestReplyMessage(control_bridge_ptr, "request_system_name");
    if (reply.IsInitialized()) {
      if (
        param::HasKey(reply, "request_system_name") &&
        param::GetValue(reply, "request_system_name").type() == cloisim::msgs::Any_ValueType_STRING &&
        !param::GetValue(reply, "request_system_name").string_value().empty())
      {
        const auto system_name = param::GetValue(reply, "request_system_name").string_value();

        param::SetName(request_msg_, system_name);
      } else {
        param::SetName(request_msg_, "ElevatorSystem");
      }
    }
  }

  srvCallElevator_ = this->create_service<CallElevator>(
    nodeName + string("/call_elevator"), BindCallback(DoElevatorCalling));

  srvGetCalledElevator_ = this->create_service<CallElevator>(
    nodeName + string("/get_called_elevator"), BindCallback(GetElevatorCalled));

  srvGetElevatorInfo_ = this->create_service<GetElevatorInformation>(
    nodeName + string("/get_elevator_information"), BindCallback(GetElevatorInfo));

  srvSelectElevatorFloor_ = this->create_service<SelectElevatorFloor>(
    nodeName + string("/select_elevator_floor"), BindCallback(SelectFloor));

  srvRequestDoorOpen_ = this->create_service<RequestDoor>(
    nodeName + string("/request_door_open"), BindCallback(RequestDoorOpen));

  srvRequestDoorClose_ = this->create_service<RequestDoor>(
    nodeName + string("/request_door_close"), BindCallback(RequestDoorClose));

  srvIsDoorOpened_ = this->create_service<RequestDoor>(
    nodeName + string("/is_door_opened"), BindCallback(IsDoorOpened));

  srvReserveElevator_ = this->create_service<ReturnBool>(
    nodeName + string("/reserve_elevator"), BindCallback(ReserveElevator));

  srvReleaseElevator_ = this->create_service<ReturnBool>(
    nodeName + string("/release_elevator"), BindCallback(ReleaseElevator));
}

void ElevatorSystem::DoElevatorCalling(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<CallElevator::Request> request,
  const shared_ptr<CallElevator::Response> response)
{
  const auto message =
    CreateRequest("call_elevator", request->current_floor, request->target_floor);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::GetElevatorCalled(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<CallElevator::Request> request,
  const std::shared_ptr<CallElevator::Response> response)
{
  auto message =
    CreateRequest("get_called_elevator", request->current_floor, request->target_floor);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    response->result = GetResultFromResponse(reply);

    const auto result_param = reply.children(2);
    if (result_param.IsInitialized()) {
      const auto elevator_index = (!param::HasKey(result_param, "elevator_index")) ?
        "" :
        param::GetValue(result_param, "elevator_index").string_value();
      response->elevator_index = elevator_index;
    }
  }
}

void ElevatorSystem::GetElevatorInfo(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<GetElevatorInformation::Request> request,
  const std::shared_ptr<GetElevatorInformation::Response> response)
{
  auto message = CreateRequest("get_elevator_information", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    cloisim::msgs::Param result_param;

    response->result = GetResultFromResponse(reply);

    result_param = reply.children(3);
    if (result_param.IsInitialized()) {
      const auto current_floor = (!param::HasKey(result_param, "current_floor")) ?
        "" :
        param::GetValue(result_param, "current_floor").string_value();
      response->current_floor = current_floor;
    }

    result_param = reply.children(4);
    if (result_param.IsInitialized()) {
      const auto height = static_cast<float>(
        (!param::HasKey(result_param, "height")) ? 0.0 : param::GetValue(result_param, "height").double_value());
      response->height = height;
    }
  }
}

void ElevatorSystem::SelectFloor(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SelectElevatorFloor::Request> request,
  const std::shared_ptr<SelectElevatorFloor::Response> response)
{
  auto message = CreateRequest(
    "select_elevator_floor", request->current_floor, request->target_floor,
    request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::RequestDoorOpen(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<RequestDoor::Request> request,
  const std::shared_ptr<RequestDoor::Response> response)
{
  auto message = CreateRequest("request_door_open", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::RequestDoorClose(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<RequestDoor::Request> request,
  const std::shared_ptr<RequestDoor::Response> response)
{
  auto message = CreateRequest("request_door_close", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::IsDoorOpened(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<RequestDoor::Request> request,
  const std::shared_ptr<RequestDoor::Response> response)
{
  auto message = CreateRequest("is_door_opened", request->elevator_index);

  const auto reply = RequestReplyMessage(control_bridge_ptr, message);
  if (reply.IsInitialized()) {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::ReserveElevator(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ReturnBool::Request>/*request*/,
  const std::shared_ptr<ReturnBool::Response> response)
{
  response->result = srv_mode_;
}

void ElevatorSystem::ReleaseElevator(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ReturnBool::Request>/*request*/,
  const std::shared_ptr<ReturnBool::Response> response)
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
cloisim::msgs::Param ElevatorSystem::CreateRequest(
  const string service_name, const string current_floor, const string target_floor,
  const string elevator_index)
{
  request_msg_.clear_children();

  cloisim::msgs::Any str_val;
  str_val.set_type(cloisim::msgs::Any::STRING);

  auto * child_ptr = request_msg_.add_children();
  str_val.set_string_value(service_name);
  param::Set(*child_ptr, "service_name", str_val);

  child_ptr = request_msg_.add_children();
  str_val.set_string_value(current_floor);
  param::Set(*child_ptr, "current_floor", str_val);

  child_ptr = request_msg_.add_children();
  str_val.set_string_value(target_floor);
  param::Set(*child_ptr, "target_floor", str_val);

  child_ptr = request_msg_.add_children();
  str_val.set_string_value(elevator_index);
  param::Set(*child_ptr, "elevator_index", str_val);

  return request_msg_;
}

bool ElevatorSystem::GetResultFromResponse(
  const cloisim::msgs::Param & response_msg, const int children_index)
{
  const auto result_param = response_msg.children(children_index);
  if (!result_param.IsInitialized() || !param::HasKey(result_param, "result")) {
    return false;
  }

  return param::GetValue(result_param, "result").bool_value();
}

}  // namespace cloisim_ros
