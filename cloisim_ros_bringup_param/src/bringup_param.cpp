/**
 *  @file   main.cpp
 *  @date   2021-01-16
 *  @author Hyunseok Yang
 *  @brief
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_bringup_param/bringup_param.hpp"

using namespace cloisim_ros;
using namespace std;

BringUpParam::BringUpParam(const std::string basename)
    : Node(basename + "_param",
           rclcpp::NodeOptions()
               .automatically_declare_parameters_from_overrides(true)
               .allow_undeclared_parameters(false)),
      isSingleMode(false), target_model(""), target_parts_type(""), target_parts_name("")
{
  get_parameter("single_mode", isSingleMode);
  get_parameter("target_model", target_model);
  get_parameter("target_parts_type", target_parts_type);
  get_parameter("target_parts_name", target_parts_name);

  RCLCPP_INFO_ONCE(this->get_logger(),
                   "single mode: %d target model: %s target_parts_type: %s target_parts_name: %s",
                   isSingleMode, target_model.c_str(), target_parts_type.c_str(), target_parts_name.c_str());
}