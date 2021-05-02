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

namespace cloisim_ros
{
  class BringUpParam : public rclcpp::Node
  {
  public:
    BringUpParam(const std::string basename = "cloisim_ros");

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
}
#endif