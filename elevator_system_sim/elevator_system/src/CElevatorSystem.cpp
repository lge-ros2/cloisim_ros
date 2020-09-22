/**
 *  @file   CElevatorSystem.cpp
 *  @date   2020-04-22
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

#include "elevator_system/CElevatorSystem.hpp"

using namespace std;
using namespace placeholders;

CElevatorSystem::CElevatorSystem(bool intra_process_comms)
    : LifecycleNode("elevator_system",
                    rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)
                        .use_intra_process_comms(intra_process_comms))
    , m_pSimBridge(new SimBridge())
    , systemName_("ElevatorSystem_")
    , srvMode_(false)
{
  string model_name;
  get_parameter_or("model", model_name, string("SeochoTower"));

  m_hashKey = model_name + get_name();
   DBG_SIM_MSG("TAG=[%s]", m_hashKey.c_str());
}

CallbackReturn CElevatorSystem::on_configure(const State &)
{
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  uint16_t portControl;
  get_parameter_or("bridge.Control", portControl, uint16_t(0));

  m_pSimBridge->Connect(SimBridge::Mode::CLIENT, portControl, m_hashKey + "Control");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CElevatorSystem::on_activate(const State &)
{
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  msgs::Param newRequest;
  newRequest.set_name("request_system_name");

  if (send_request(newRequest))
  {
    msgs::Param response;
    if (receive_response(response))
    {
      if (response.name().compare("request_system_name") == 0)
      {
        systemName_ = response.value().string_value();
      }
    }
  }

  auto nodeName = get_name();

  m_pSrvCallElevator = this->create_service<CallElevator>(
      nodeName + string("/call_elevator"),
      bind(&CElevatorSystem::callback_call_elevator, this, _1, _2, _3));

  m_pSrvGetCalledElevator = this->create_service<CallElevator>(
      nodeName + string("/get_called_elevator"),
      bind(&CElevatorSystem::callback_get_called_elevator, this, _1, _2, _3));

  m_pSrvGetElevatorInformation = this->create_service<GetElevatorInfo>(
      nodeName + string("/get_elevator_information"),
      bind(&CElevatorSystem::callback_get_elevator_information, this, _1, _2, _3));

  m_pSrvSelectElevatorFloor = this->create_service<SelectElevatorFloor>(
      nodeName + string("/select_elevator_floor"),
      bind(&CElevatorSystem::callback_select_elevator_floor, this, _1, _2, _3));

  m_pSrvRequestDoorOpen = this->create_service<RequestDoor>(
      nodeName + string("/request_door_open"),
      bind(&CElevatorSystem::callback_request_door_open, this, _1, _2, _3));

  m_pSrvRequestDoorClose = this->create_service<RequestDoor>(
      nodeName + string("/request_door_close"),
      bind(&CElevatorSystem::callback_request_door_close, this, _1, _2, _3));

  m_pSrvIsDoorOpened = this->create_service<RequestDoor>(
      nodeName + string("/is_door_opened"),
      bind(&CElevatorSystem::callback_is_door_opened, this, _1, _2, _3));

  m_pSrvReserveElevator = this->create_service<ReturnBool>(
      nodeName + string("/reserve_elevator"),
      bind(&CElevatorSystem::callback_reserve_elevator, this, _1, _2, _3));

  m_pSrvReleaseElevator = this->create_service<ReturnBool>(
      nodeName + string("/release_elevator"),
      bind(&CElevatorSystem::callback_release_elevator, this, _1, _2, _3));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CElevatorSystem::on_deactivate(const State &)
{
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CElevatorSystem::on_cleanup(const State &)
{
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CElevatorSystem::on_shutdown(const State &)
{
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CElevatorSystem::on_error(const State &)
{
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return CallbackReturn::SUCCESS;
}

void CElevatorSystem::callback_call_elevator(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<CallElevator::Request> request,
    const shared_ptr<CallElevator::Response> response)
{
  auto message = create_request_message("call_elevator",
                                            request->current_floor,
                                            request->target_floor);

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
    }
  }
}

void CElevatorSystem::callback_get_called_elevator(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<CallElevator::Request> request,
    const shared_ptr<CallElevator::Response> response)
{
  auto message = create_request_message("get_called_elevator",
                                            request->current_floor,
                                            request->target_floor);

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
      response->elevator_index = to_string(get_elevator_index_from_response_message(response_msg));
    }
  }
}

void CElevatorSystem::callback_get_elevator_information(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<GetElevatorInfo::Request> request,
    const shared_ptr<GetElevatorInfo::Response> response)
{
  auto message = create_request_message("get_elevator_information", stoi(request->elevator_index));

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
      response->current_floor = get_current_floor_from_response_message(response_msg);
      response->height = get_height_from_response_message(response_msg);
    }
  }
}

void CElevatorSystem::callback_select_elevator_floor(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<SelectElevatorFloor::Request> request,
    const shared_ptr<SelectElevatorFloor::Response> response)
{
  auto message = create_request_message("select_elevator_floor",
                                            request->current_floor,
                                            request->target_floor,
                                            stoi(request->elevator_index));

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
    }
  }
}

void CElevatorSystem::callback_request_door_open(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<RequestDoor::Request> request,
    const shared_ptr<RequestDoor::Response> response)
{
  auto message = create_request_message("request_door_open", stoi(request->elevator_index));

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
    }
  }
}

void CElevatorSystem::callback_request_door_close(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<RequestDoor::Request> request,
    const shared_ptr<RequestDoor::Response> response)
{
  auto message = create_request_message("request_door_close", stoi(request->elevator_index));

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
    }
  }
}

void CElevatorSystem::callback_is_door_opened(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<RequestDoor::Request> request,
    const shared_ptr<RequestDoor::Response> response)
{
  auto message = create_request_message("is_door_opened", stoi(request->elevator_index));

  if (send_request(message))
  {
    msgs::Param response_msg;
    if (receive_response(response_msg))
    {
      response->result = get_result_from_response_message(response_msg);
    }
  }
}

void CElevatorSystem::callback_reserve_elevator(
  const shared_ptr<rmw_request_id_t> /*request_header*/,
  const shared_ptr<ReturnBool::Request> /*request*/,
  const shared_ptr<ReturnBool::Response> response)
{
  response->result = srvMode_;
}

void CElevatorSystem::callback_release_elevator(
  const shared_ptr<rmw_request_id_t> /*request_header*/,
  const shared_ptr<ReturnBool::Request> /*request*/,
  const shared_ptr<ReturnBool::Response> response)
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
msgs::Param CElevatorSystem::create_request_message(
    string service_name,
    string current_floor,
    string target_floor,
    int elevator_index)
{
  msgs::Param newMessage;
  newMessage.set_name(systemName_);

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
  pVal->set_type(msgs::Any::INT32);
  pVal->set_int_value(elevator_index);

  return newMessage;
}

bool CElevatorSystem::get_result_from_response_message(const msgs::Param &response_msg)
{
  auto result_param = response_msg.children(1);
  if (!result_param.IsInitialized() || result_param.name().compare("result") != 0)
  {
    return false;
  }

  return result_param.value().bool_value();
}

int CElevatorSystem::get_elevator_index_from_response_message(const msgs::Param &response_msg)
{
  auto result_param = response_msg.children(2);
  if (!result_param.IsInitialized() || result_param.name().compare("elevator_index") != 0)
  {
    return NON_ELEVATOR_INDEX;
  }

  return result_param.value().int_value();
}

string CElevatorSystem::get_current_floor_from_response_message(const msgs::Param &response_msg)
{
  auto result_param = response_msg.children(3);
  if (!result_param.IsInitialized() || result_param.name().compare("current_floor") != 0)
  {
    return "";
  }

  return result_param.value().string_value();
}

float CElevatorSystem::get_height_from_response_message(const msgs::Param &response_msg)
{
  auto result_param = response_msg.children(4);
  if (!result_param.IsInitialized() || result_param.name().compare("height") != 0)
  {
    return false;
  }

  return (float)result_param.value().double_value();
}

/**
 * @brief Send request
 * @param request_msg[IN]   request context
 * @return successful or failed
 */
bool CElevatorSystem::send_request(const msgs::Param &request_msg)
{
  const lock_guard<mutex> lock(m_mtxSendRequest);

  string serializedBuffer;
  request_msg.SerializeToString(&serializedBuffer);

  return m_pSimBridge->Send(serializedBuffer.data(), serializedBuffer.size());
}

/**
 * @brief Receive response
 * @param response_msg[OUT]   response context
 * @return successful or failed
 */
bool CElevatorSystem::receive_response(msgs::Param &response_msg)
{
  const lock_guard<mutex> lock(m_mtxReceiveResponse);

  void *pBuffer = nullptr;
  int bufferLength = 0;

  const auto succeeded = m_pSimBridge->Receive(&pBuffer, bufferLength);

  if (succeeded)
  {
    return response_msg.ParseFromArray(pBuffer, bufferLength);
  }

  return false;
}


