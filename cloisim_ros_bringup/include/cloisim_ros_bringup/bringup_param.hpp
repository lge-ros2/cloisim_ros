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
    BringUpParam(const std::string basename = "cloisim_ros")
        : Node(basename + "_param",
               rclcpp::NodeOptions()
                   .automatically_declare_parameters_from_overrides(true)
                   .allow_undeclared_parameters(false))
        , isSingleMode(false)
        , target_model("")
        , target_parts_type("")
        , target_parts_name("")
    {
      get_parameter("single_mode", isSingleMode);
      get_parameter("target_model", target_model);
      get_parameter("target_parts_type", target_parts_type);
      get_parameter("target_parts_name", target_parts_name);

      RCLCPP_INFO_ONCE(this->get_logger(),
                       "single mode: %d target model: %s target_parts_type: %s target_parts_name: %s",
                       isSingleMode, target_model.c_str(), target_parts_type.c_str(), target_parts_name.c_str());
    };

    bool IsSingleMode() const { return isSingleMode; }
    std::string TargetModel() const { return target_model; }
    std::string TargetPartsType() const { return target_parts_type; }
    std::string TargetPartsName() const { return target_parts_name; }

  private:
    bool isSingleMode;
    std::string target_model;
    std::string target_parts_type;
    std::string target_parts_name;
};

#endif