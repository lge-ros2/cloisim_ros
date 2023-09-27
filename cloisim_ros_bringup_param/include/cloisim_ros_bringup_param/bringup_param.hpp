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

#ifndef CLOISIM_ROS_BRINGUP_PARAM__BRINGUP_PARAM_HPP_
#define CLOISIM_ROS_BRINGUP_PARAM__BRINGUP_PARAM_HPP_

#include <jsoncpp/json/json.h>

#include <string>

#include <cloisim_ros_websocket_service/websocket_service.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;

namespace cloisim_ros
{

class BringUpParam : public rclcpp::Node
{
 public:
  explicit BringUpParam(const string basename = "cloisim_ros");
  virtual ~BringUpParam();

  bool IsSingleMode() const { return is_single_mode; }
  string TargetModel() const { return target_model; }
  string TargetPartsType() const { return target_parts_type; }
  string TargetPartsName() const { return target_parts_name; }

  void IsSingleMode(const bool value) { is_single_mode = value; }
  void TargetModel(const string value) { target_model = value; }
  void TargetPartsType(const string value) { target_parts_type = value; }
  void TargetPartsName(const string value) { target_parts_name = value; }

  Json::Value GetBringUpList(const bool filterByParameters = false);

  void StoreFilteredInfoAsParameters(const Json::Value item, rclcpp::NodeOptions &node_options);

 public:
  Json::Value RequestBringUpList();

  static bool IsRobotSpecificType(const string node_type);

  static bool IsWorldSpecificType(const string node_type);

  static void StoreBridgeInfosAsParameters(const Json::Value item, rclcpp::NodeOptions &node_options);

 private:
  bool is_single_mode;
  string target_model;
  string target_parts_type;
  string target_parts_name;

  WebSocketService *ws_service_ptr_;

 private:
  Json::Value GetFilteredListByParameters(const Json::Value result);
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_BRINGUP_PARAM__BRINGUP_PARAM_HPP_
