/**
 *  @file   joint_control.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 JointControl class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#ifndef _CLOISIM_ROS_JOINTCONTROL_HPP__
#define _CLOISIM_ROS_JOINTCONTROL_HPP__

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cloisim_msgs/joint_cmd.pb.h>
#include <cloisim_msgs/joint_state_v.pb.h>

namespace cloisim_ros
{
  class JointControl : public Base
  {
  public:
    explicit JointControl(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
    explicit JointControl(const std::string namespace_ = "");
    virtual ~JointControl();

  private:
    void Initialize() override;
    void Deinitialize() override { };
    void UpdatePublishingData(const std::string &buffer) override;

  private:
    void GetTransformNameInfo(zmq::Bridge *const bridge_ptr);

    void JointControlWrite(zmq::Bridge* const bridge_ptr, const std::string &buffer);

    std::string MakeCommandMessage(const control_msgs::msg::JointJog::SharedPtr msg) const;

  private:
    zmq::Bridge *info_bridge_ptr;

    std::map<std::string, std::string> target_transform_name;

    // JointControl msgs
    cloisim::msgs::JointCmd pb_joint_cmd_;
    cloisim::msgs::JointState_V pb_joint_states;

    // jointstate msgs
    sensor_msgs::msg::JointState msg_jointstate_;

    // ROS2 JointControl publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;

    // ROS2 Joint Jog subscriber
    rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr sub_joint_job__;
  };
}
#endif
