/**
 *  @file   CElevatorSystem.hpp
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

#ifndef _CELEVATOR_SYSTEM_H_
#define _CELEVATOR_SYSTEM_H_

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sim_bridge/sim_bridge.hpp>
#include <protobuf/param.pb.h>

#include <elevator_system_msgs/srv/call_elevator.hpp>
#include <elevator_system_msgs/srv/get_elevator_information.hpp>
#include <elevator_system_msgs/srv/select_elevator_floor.hpp>
#include <elevator_system_msgs/srv/request_door.hpp>
#include <elevator_system_msgs/srv/return_bool.hpp>

using CallElevator = elevator_system_msgs::srv::CallElevator;
using GetElevatorInfo = elevator_system_msgs::srv::GetElevatorInformation;
using SelectElevatorFloor = elevator_system_msgs::srv::SelectElevatorFloor;
using RequestDoor = elevator_system_msgs::srv::RequestDoor;
using ReturnBool = elevator_system_msgs::srv::ReturnBool;

using namespace std::chrono_literals;
using namespace gazebo;
using namespace rclcpp_lifecycle;
using CallbackReturn = node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CElevatorSystem : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CElevatorSystem(bool intra_process_comms = false);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
  CallbackReturn on_error(const rclcpp_lifecycle::State &);

private:
  SimBridge *m_pSimBridge;
  std::string systemName_;
  std::string m_hashKey;

  rclcpp::Service<CallElevator>::SharedPtr m_pSrvCallElevator;
  rclcpp::Service<CallElevator>::SharedPtr m_pSrvGetCalledElevator;
  rclcpp::Service<GetElevatorInfo>::SharedPtr m_pSrvGetElevatorInformation;
  rclcpp::Service<SelectElevatorFloor>::SharedPtr m_pSrvSelectElevatorFloor;
  rclcpp::Service<RequestDoor>::SharedPtr m_pSrvRequestDoorOpen;
  rclcpp::Service<RequestDoor>::SharedPtr m_pSrvRequestDoorClose;
  rclcpp::Service<RequestDoor>::SharedPtr m_pSrvIsDoorOpened;
  rclcpp::Service<ReturnBool>::SharedPtr m_pSrvReserveElevator;
  rclcpp::Service<ReturnBool>::SharedPtr m_pSrvReleaseElevator;

  std::mutex m_mtxSendRequest;
  std::mutex m_mtxReceiveResponse;

private:

  void callback_call_elevator(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<CallElevator::Request> request,
                              const std::shared_ptr<CallElevator::Response> response);

  void callback_get_called_elevator(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<CallElevator::Request> request,
                                    const std::shared_ptr<CallElevator::Response> response);

  void callback_get_elevator_information(const std::shared_ptr<rmw_request_id_t> request_header,
                                         const std::shared_ptr<GetElevatorInfo::Request> request,
                                         const std::shared_ptr<GetElevatorInfo::Response> response);

  void callback_select_elevator_floor(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SelectElevatorFloor::Request> request,
                                 const std::shared_ptr<SelectElevatorFloor::Response> response);

  void callback_request_door_open(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<RequestDoor::Request> request,
                                   const std::shared_ptr<RequestDoor::Response> response);

  void callback_request_door_close(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<RequestDoor::Request> request,
                                      const std::shared_ptr<RequestDoor::Response> response);

  void callback_is_door_opened(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<RequestDoor::Request> request,
                               const std::shared_ptr<RequestDoor::Response> response);

  void callback_reserve_elevator(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<ReturnBool::Request> request,
                               const std::shared_ptr<ReturnBool::Response> response);

  void callback_release_elevator(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<ReturnBool::Request> request,
                               const std::shared_ptr<ReturnBool::Response> response);

  msgs::Param create_request_message(std::string service_name, std::string elevator_index);

  msgs::Param create_request_message(std::string service_name,
                                     std::string current_floor,
                                     std::string target_floor,
                                     std::string elevator_index = "");

  bool get_result_from_response_message(const msgs::Param &response_msg);
  std::string get_elevator_index_from_response_message(const msgs::Param &response_msg);
  std::string get_current_floor_from_response_message(const msgs::Param &response_msg);
  float get_height_from_response_message(const msgs::Param &response_msg);

  bool srvMode_;

  bool send_request(const msgs::Param &request_msg);
  bool receive_response(msgs::Param &response_msg);
};

inline msgs::Param CElevatorSystem::create_request_message(std::string service_name, std::string elevator_index)
{
  return create_request_message(service_name, "", "", elevator_index);
}

#endif