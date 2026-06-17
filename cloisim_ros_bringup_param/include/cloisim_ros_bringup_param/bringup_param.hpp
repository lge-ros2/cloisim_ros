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

#include <cstdio>
#include <memory>
#include <stdexcept>
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

  bool IsSingleMode() const {return is_single_mode;}
  std::string TargetModel() const {return target_model;}
  std::string TargetPartsType() const {return target_parts_type;}
  std::string TargetPartsName() const {return target_parts_name;}

  void IsSingleMode(const bool value) {is_single_mode = value;}
  void TargetModel(const std::string value) {target_model = value;}
  void TargetPartsType(const std::string value) {target_parts_type = value;}
  void TargetPartsName(const std::string value) {target_parts_name = value;}

  Json::Value GetBringUpList(const bool filterByParameters = false);
  Json::Value RequestBringUpList();

  static bool IsRobotSpecificType(const std::string node_type);
  static bool IsWorldSpecificType(const std::string node_type);

  void StoreFilteredInfoAsParameters(const Json::Value item, rclcpp::NodeOptions & node_options);

  static void StoreBridgeInfosAsParameters(
    const Json::Value item, rclcpp::NodeOptions & node_options);

  bool HasEnableTFforMicom() {return has_parameter("micom.enable_tf");}
  bool EnableTFforMicom() {return enable_tf_micom_;}

private:
  static constexpr const char * MIN_CLOISIM_VERSION = "5.3.1";

  void SetIndividualParameters(Json::Value & result);
  void CheckSimulatorVersion(const Json::Value & root);
  static int CompareVersions(const string & a, const string & b);

private:
  bool is_single_mode;
  std::string target_model;
  std::string target_parts_type;
  std::string target_parts_name;
  bool enable_tf_micom_;

  std::unique_ptr<WebSocketService> ws_service_ptr_;

private:
  Json::Value GetFilteredListByParameters(const Json::Value result);
};
}  // namespace cloisim_ros

// ---------------------------------------------------------------------------
// Convenience launcher templates used by device-node main() functions.
// ---------------------------------------------------------------------------

namespace cloisim_ros
{

// Standard multi/single-mode device: NodeT(options, name) or NodeT(options, name, model).
template<typename NodeT>
int RunNode(int argc, char ** argv, const char * pkg_name, const char * parts_type)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  const auto param_node = std::make_shared<cloisim_ros::BringUpParam>(pkg_name);
  param_node->TargetPartsType(parts_type);
  executor.add_node(param_node);

  const auto filtered = param_node->GetBringUpList(true);

  if (!filtered.empty()) {
    rclcpp::NodeOptions opts;
    param_node->StoreFilteredInfoAsParameters(filtered, opts);
    const auto node_name = param_node->TargetPartsName();
    std::shared_ptr<NodeT> node;
    if (param_node->IsSingleMode()) {
      node = std::make_shared<NodeT>(opts, node_name);
    } else {
      node = std::make_shared<NodeT>(opts, node_name, param_node->TargetModel());
    }
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// Single-mode-only device (World, GroundTruth, …): NodeT(options, name).
template<typename NodeT>
int RunNodeSingleMode(int argc, char ** argv, const char * pkg_name, const char * parts_type)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  const auto param_node = std::make_shared<cloisim_ros::BringUpParam>(pkg_name);
  param_node->IsSingleMode(true);
  param_node->TargetPartsType(parts_type);
  executor.add_node(param_node);

  const auto filtered = param_node->GetBringUpList(true);

  if (!filtered.empty()) {
    rclcpp::NodeOptions opts;
    param_node->StoreFilteredInfoAsParameters(filtered, opts);
    auto node = std::make_shared<NodeT>(opts, param_node->TargetPartsName());
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}

}  // namespace cloisim_ros (launcher templates)

#endif  // CLOISIM_ROS_BRINGUP_PARAM__BRINGUP_PARAM_HPP_
