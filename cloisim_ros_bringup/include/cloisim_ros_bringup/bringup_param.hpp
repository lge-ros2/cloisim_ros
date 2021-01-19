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
                   .allow_undeclared_parameters(false)
                   .use_intra_process_comms(false))
        , isSingleMode(false)
    {
      get_parameter_or("singlemode", isSingleMode, bool(false));

      RCLCPP_INFO_ONCE(this->get_logger(), "singleMode: %d", isSingleMode);
    };

    bool IsSingleMode() const { return isSingleMode; }

  private:
    bool isSingleMode;
};

#endif