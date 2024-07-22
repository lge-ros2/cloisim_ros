/**
 *  @file   bringup_param.hpp
 *  @date   2024-06-24
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

using string = std::string;

namespace cloisim_ros
{
class BringUpParam : public rclcpp::Node
{
 public:
  explicit BringUpParam(const std::string basename = "cloisim_ros");
  virtual ~BringUpParam();

  bool IsSingleMode() const { return is_single_mode; }
  std::string TargetModel() const { return target_model; }
  std::string TargetPartsType() const { return target_parts_type; }
  std::string TargetPartsName() const { return target_parts_name; }

  void IsSingleMode(const bool value) { is_single_mode = value; }
  void TargetModel(const std::string value) { target_model = value; }
  void TargetPartsType(const std::string value) { target_parts_type = value; }
  void TargetPartsName(const std::string value) { target_parts_name = value; }

  Json::Value GetBringUpList(const bool filterByParameters = false);
  Json::Value RequestBringUpList();

  static bool IsRobotSpecificType(const std::string node_type);
  static bool IsWorldSpecificType(const std::string node_type);

  void StoreFilteredInfoAsParameters(
      const Json::Value item, rclcpp::NodeOptions &node_options);

  static void StoreBridgeInfosAsParameters(
      const Json::Value item, rclcpp::NodeOptions &node_options);

  bool HasEnableTFforMicom() { return has_parameter("micom.enable_tf"); }
  bool EnableTFforMicom() { return enable_tf_micom_; }

private:
  void SetIndividualParameters(Json::Value& result);

 private:
  bool is_single_mode;
  std::string target_model;
  std::string target_parts_type;
  std::string target_parts_name;
  bool enable_tf_micom_;

  WebSocketService *ws_service_ptr_;

 private:
  Json::Value GetFilteredListByParameters(const Json::Value result);
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_BRINGUP_PARAM__BRINGUP_PARAM_HPP_
