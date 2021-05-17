/**
 *  @file   elevatorsystems.hpp
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

#ifndef _CLOISIM_ROS_ELEVATORSYSTEM_HPP_
#define _CLOISIM_ROS_ELEVATORSYSTEM_HPP_

#include <cloisim_ros_base/base.hpp>
#include <cloisim_msgs/param.pb.h>
#include <elevator_system_msgs/srv/call_elevator.hpp>
#include <elevator_system_msgs/srv/get_elevator_information.hpp>
#include <elevator_system_msgs/srv/select_elevator_floor.hpp>
#include <elevator_system_msgs/srv/request_door.hpp>
#include <elevator_system_msgs/srv/return_bool.hpp>

namespace cloisim_ros
{
  class ElevatorSystem : public Base
  {
  public:
    explicit ElevatorSystem(const rclcpp::NodeOptions &options_, const std::string node_name_);
    explicit ElevatorSystem();
    virtual ~ElevatorSystem();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdatePublishingData(const std::string &buffer) override { (void)buffer; };

  private:
    std::string systemName;

    zmq::Bridge *pBridgeControl;

    bool srvMode_;

    rclcpp::Service<elevator_system_msgs::srv::CallElevator>::SharedPtr srvCallElevator_;
    rclcpp::Service<elevator_system_msgs::srv::CallElevator>::SharedPtr srvGetCalledElevator_;
    rclcpp::Service<elevator_system_msgs::srv::GetElevatorInformation>::SharedPtr srvGetElevatorInfo_;
    rclcpp::Service<elevator_system_msgs::srv::SelectElevatorFloor>::SharedPtr srvSelectElevatorFloor_;
    rclcpp::Service<elevator_system_msgs::srv::RequestDoor>::SharedPtr srvRequestDoorOpen_;
    rclcpp::Service<elevator_system_msgs::srv::RequestDoor>::SharedPtr srvRequestDoorClose_;
    rclcpp::Service<elevator_system_msgs::srv::RequestDoor>::SharedPtr srvIsDoorOpened_;
    rclcpp::Service<elevator_system_msgs::srv::ReturnBool>::SharedPtr srvReserveElevator_;
    rclcpp::Service<elevator_system_msgs::srv::ReturnBool>::SharedPtr srvReleaseElevator_;

  private:
    void CallElevator(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                      const std::shared_ptr<elevator_system_msgs::srv::CallElevator::Request> /*request*/,
                      const std::shared_ptr<elevator_system_msgs::srv::CallElevator::Response> response);

    void GetElevatorCalled(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                           const std::shared_ptr<elevator_system_msgs::srv::CallElevator::Request> request,
                           const std::shared_ptr<elevator_system_msgs::srv::CallElevator::Response> response);

    void GetElevatorInfo(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                         const std::shared_ptr<elevator_system_msgs::srv::GetElevatorInformation::Request> request,
                         const std::shared_ptr<elevator_system_msgs::srv::GetElevatorInformation::Response> response);

    void SelectFloor(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                     const std::shared_ptr<elevator_system_msgs::srv::SelectElevatorFloor::Request> request,
                     const std::shared_ptr<elevator_system_msgs::srv::SelectElevatorFloor::Response> response);

    void RequestDoorOpen(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                           const std::shared_ptr<elevator_system_msgs::srv::RequestDoor::Request> request,
                           const std::shared_ptr<elevator_system_msgs::srv::RequestDoor::Response> response);

    void RequestDoorClose(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                            const std::shared_ptr<elevator_system_msgs::srv::RequestDoor::Request> request,
                            const std::shared_ptr<elevator_system_msgs::srv::RequestDoor::Response> response);

    void IsDoorOpened(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                        const std::shared_ptr<elevator_system_msgs::srv::RequestDoor::Request> request,
                        const std::shared_ptr<elevator_system_msgs::srv::RequestDoor::Response> response);

    void ReserveElevator(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                          const std::shared_ptr<elevator_system_msgs::srv::ReturnBool::Request> /*request*/,
                          const std::shared_ptr<elevator_system_msgs::srv::ReturnBool::Response> response);

    void ReleaseElevator(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                          const std::shared_ptr<elevator_system_msgs::srv::ReturnBool::Request> /*request*/,
                          const std::shared_ptr<elevator_system_msgs::srv::ReturnBool::Response> response);

    cloisim::msgs::Param CreateRequest(const std::string service_name, const std::string elevator_index)
    {
      return CreateRequest(service_name, "", "", elevator_index);
    }

    cloisim::msgs::Param CreateRequest(const std::string service_name,
                                       const std::string current_floor,
                                       const std::string target_floor,
                                       const std::string elevator_index = "");

    bool GetResultFromResponse(const cloisim::msgs::Param &response_msg, const int children_index = 1);
  };
}
#endif