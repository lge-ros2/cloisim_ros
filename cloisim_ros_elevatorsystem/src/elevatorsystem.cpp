/**
 *  @file   elevatorsystem.cpp
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

#include "cloisim_ros_elevatorsystem/elevatorsystem.hpp"

using namespace std;
using namespace placeholders;
using namespace cloisim;
using namespace cloisim_ros;
using namespace elevator_system_msgs;

#define BindCallback(func) bind(&ElevatorSystem::func, this, _1, _2, _3)

ElevatorSystem::ElevatorSystem(const rclcpp::NodeOptions &options_, const string node_name_)
    : Base(node_name_, options_)
    , systemName("ElevatorSystem")
    , pBridgeControl(nullptr)
    , srvMode_(false)
{
  Start(false);
}

ElevatorSystem::ElevatorSystem()
    : ElevatorSystem(rclcpp::NodeOptions(), "cloisim_ros_elevatorsystem")
{
}

ElevatorSystem::~ElevatorSystem()
{
  Stop();
}

void ElevatorSystem::Initialize()
{
  const auto nodeName = GetPartsName();
  const auto hashKeySrv = GetModelName() + nodeName + "Control";
  DBG_SIM_INFO("hash Key srv: %s", hashKeySrv.c_str());

  uint16_t portControl;
  get_parameter_or("bridge.Control", portControl, uint16_t(0));

  pBridgeControl = CreateBridge(hashKeySrv);
  if (pBridgeControl != nullptr)
  {
    pBridgeControl->Connect(zmq::Bridge::Mode::CLIENT, portControl, hashKeySrv);

    const auto reply = RequestReplyMessage(pBridgeControl, "request_system_name");
    if (reply.IsInitialized())
    {
      if (reply.name().compare("request_system_name") == 0 &&
          reply.value().type() == msgs::Any_ValueType_STRING &&
          !reply.value().string_value().empty())
      {
        systemName = reply.value().string_value();
      }
    }
  }

  srvCallElevator_ = this->create_service<srv::CallElevator>(
      nodeName + string("/call_elevator"), BindCallback(CallElevator));

  srvGetCalledElevator_ = this->create_service<srv::CallElevator>(
      nodeName + string("/get_called_elevator"), BindCallback(GetElevatorCalled));

  srvGetElevatorInfo_ = this->create_service<srv::GetElevatorInformation>(
      nodeName + string("/get_elevator_information"), BindCallback(GetElevatorInfo));

  srvSelectElevatorFloor_ = this->create_service<srv::SelectElevatorFloor>(
      nodeName + string("/select_elevator_floor"), BindCallback(SelectFloor));

  srvRequestDoorOpen_ = this->create_service<srv::RequestDoor>(
      nodeName + string("/request_door_open"), BindCallback(RequestDoorOpen));

  srvRequestDoorClose_ = this->create_service<srv::RequestDoor>(
      nodeName + string("/request_door_close"), BindCallback(RequestDoorClose));

  srvIsDoorOpened_ = this->create_service<srv::RequestDoor>(
      nodeName + string("/is_door_opened"), BindCallback(IsDoorOpened));

  srvReserveElevator_ = this->create_service<srv::ReturnBool>(
      nodeName + string("/reserve_elevator"), BindCallback(ReserveElevator));

  srvReleaseElevator_ = this->create_service<srv::ReturnBool>(
      nodeName + string("/release_elevator"), BindCallback(ReleaseElevator));
}

void ElevatorSystem::Deinitialize()
{
}


void ElevatorSystem::CallElevator(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::CallElevator::Request> request,
    const shared_ptr<srv::CallElevator::Response> response)
{
  const auto message = CreateRequest("call_elevator",
                               request->current_floor,
                               request->target_floor);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::GetElevatorCalled(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::CallElevator::Request> request,
    const shared_ptr<srv::CallElevator::Response> response)
{
  auto message = CreateRequest("get_called_elevator",
                               request->current_floor,
                               request->target_floor);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
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

void ElevatorSystem::GetElevatorInfo(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::GetElevatorInformation::Request> request,
    const shared_ptr<srv::GetElevatorInformation::Response> response)
{
  auto message = CreateRequest("get_elevator_information", request->elevator_index);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
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

void ElevatorSystem::SelectFloor(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::SelectElevatorFloor::Request> request,
    const shared_ptr<srv::SelectElevatorFloor::Response> response)
{
  auto message = CreateRequest("select_elevator_floor",
                               request->current_floor,
                               request->target_floor,
                               request->elevator_index);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::RequestDoorOpen(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::RequestDoor::Request> request,
    const shared_ptr<srv::RequestDoor::Response> response)
{
  auto message = CreateRequest("request_door_open", request->elevator_index);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::RequestDoorClose(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::RequestDoor::Request> request,
    const shared_ptr<srv::RequestDoor::Response> response)
{
  auto message = CreateRequest("request_door_close", request->elevator_index);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::IsDoorOpened(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<srv::RequestDoor::Request> request,
    const shared_ptr<srv::RequestDoor::Response> response)
{
  auto message = CreateRequest("is_door_opened", request->elevator_index);

  const auto reply = RequestReplyMessage(pBridgeControl, message);
  if (reply.IsInitialized())
  {
    response->result = GetResultFromResponse(reply);
  }
}

void ElevatorSystem::ReserveElevator(
  const shared_ptr<rmw_request_id_t> /*request_header*/,
  const shared_ptr<srv::ReturnBool::Request> /*request*/,
  const shared_ptr<srv::ReturnBool::Response> response)
{
  response->result = srvMode_;
}

void ElevatorSystem::ReleaseElevator(
  const shared_ptr<rmw_request_id_t> /*request_header*/,
  const shared_ptr<srv::ReturnBool::Request> /*request*/,
  const shared_ptr<srv::ReturnBool::Response> response)
{
  response->result = srvMode_;
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
msgs::Param ElevatorSystem::CreateRequest(
    const string service_name,
    const string current_floor,
    const string target_floor,
    const string elevator_index)
{
  msgs::Param newMessage;
  newMessage.set_name(systemName);

  msgs::Param *pParam;
  msgs::Any *pVal;

  pParam = newMessage.add_children();
  pParam->set_name("service_name");
  pVal = pParam->mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(service_name);

  pParam = newMessage.add_children();
  pParam->set_name("current_floor");
  pVal = pParam->mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(current_floor);

  pParam = newMessage.add_children();
  pParam->set_name("target_floor");
  pVal = pParam->mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(target_floor);

  pParam = newMessage.add_children();
  pParam->set_name("elevator_index");
  pVal = pParam->mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(elevator_index);

  return newMessage;
}

bool ElevatorSystem::GetResultFromResponse(const msgs::Param &response_msg, const int children_index)
{
  const auto result_param = response_msg.children(children_index);
  if (!result_param.IsInitialized() || result_param.name().compare("result") != 0)
  {
    return false;
  }

  return result_param.value().bool_value();
}