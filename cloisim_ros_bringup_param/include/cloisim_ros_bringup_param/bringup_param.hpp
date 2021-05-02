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
#include <cloisim_ros_websocket_service/websocket_service.hpp>
#include <jsoncpp/json/json.h>

namespace cloisim_ros
{
  class BringUpParam : public rclcpp::Node
  {
  private:
    const int maxRetryNum = 100;
    const int waitingSeconds = 3;

  public:
    BringUpParam(const string basename = "cloisim_ros");

    bool IsSingleMode() const { return isSingleMode; }
    string TargetModel() const { return target_model; }
    string TargetPartsType() const { return target_parts_type; }
    string TargetPartsName() const { return target_parts_name; }

    void IsSingleMode(const bool value) { isSingleMode = value; }
    void TargetModel(const string value) { target_model = value; }
    void TargetPartsType(const string value) { target_parts_type = value; }
    void TargetPartsName(const string value) { target_parts_name = value; }

    Json::Value GetBringUpList(const bool filterByParameters = false);

    static void StoreBridgeInfosAsParameters(const Json::Value item, rclcpp::NodeOptions &node_options);

  private:
    bool isSingleMode;
    string target_model;
    string target_parts_type;
    string target_parts_name;

    WebSocketService *wsService;

  private:
    Json::Value GetFilteredListByParameters(const Json::Value result);
  };
}
#endif