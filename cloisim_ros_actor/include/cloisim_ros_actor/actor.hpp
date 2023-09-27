/**
 *  @file   Actors.hpp
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

#ifndef CLOISIM_ROS_ACTOR__ACTOR_HPP_
#define CLOISIM_ROS_ACTOR__ACTOR_HPP_

#include <cloisim_msgs/param.pb.h>

#include <memory>
#include <string>

#include <cloisim_ros_base/base.hpp>
#include <cloisim_ros_msgs/srv/move_actor.hpp>

namespace cloisim_ros
{
class Actor : public Base
{
 public:
  explicit Actor(const rclcpp::NodeOptions &options_, const std::string node_name);
  Actor();
  virtual ~Actor();

 private:
  void Initialize() override;
  void Deinitialize() override{};

 private:
  zmq::Bridge *control_bridge_ptr;

  rclcpp::Service<cloisim_ros_msgs::srv::MoveActor>::SharedPtr srvCallMoveActor_;

 private:
  void CallMoveActor(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                     const std::shared_ptr<cloisim_ros_msgs::srv::MoveActor::Request> request,
                     const std::shared_ptr<cloisim_ros_msgs::srv::MoveActor::Response> response);

  cloisim::msgs::Param CreateMoveRequest(
      const std::string target_name,
      const geometry_msgs::msg::Vector3 point);

  bool GetResultFromResponse(const cloisim::msgs::Param &response_msg);
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_ACTOR__ACTOR_HPP_
