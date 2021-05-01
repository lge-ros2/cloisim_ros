/**
 *  @file   bringup_param.hpp
 *  @date   2021-01-21
 *  @author Hyunseok Yang
 *  @brief
 *        load parameters for bringup
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLOISIM_ROS_BRINGUP_PARAM_HPP_
#define _CLOISIM_ROS_BRINGUP_PARAM_HPP_

#include <rclcpp/rclcpp.hpp>

class BringUpParam : public rclcpp::Node
{
  public:
    BringUpParam()
        : Node("cloisim_ros_bringup_param",
               rclcpp::NodeOptions()
                   .automatically_declare_parameters_from_overrides(true)
                   .allow_undeclared_parameters(false))
        , isSingleMode(false)
        , target_model("")
        , target_parts("")
    {
      get_parameter("single_mode", isSingleMode);
      get_parameter("target_model", target_model);
      get_parameter("target_parts", target_parts);

      RCLCPP_INFO_ONCE(this->get_logger(),
                       "single mode: %d target model:%s target_parts:%s",
                       isSingleMode, target_model.c_str(), target_parts.c_str());
    };

    bool IsSingleMode() const { return isSingleMode; }
    std::string TargetModel() const { return target_model; }
    std::string TargetParts() const { return target_parts; }

  private:
    bool isSingleMode;
    std::string target_model;
    std::string target_parts;
};

#endif