/**
 *  @file   logical_camera.hpp
 *  @date   2026-04-24
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 logical camera class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_LOGICAL_CAMERA__LOGICAL_CAMERA_HPP_
#define CLOISIM_ROS_LOGICAL_CAMERA__LOGICAL_CAMERA_HPP_

#include <cloisim_msgs/logical_camera_image.pb.h>
#include <cloisim_msgs/logical_camera_sensor.pb.h>

#include <string>

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace cloisim_ros
{
class LogicalCamera : public Base
{
public:
  explicit LogicalCamera(
    const rclcpp::NodeOptions & options_, const std::string node_name,
    const std::string namespace_ = "");
  explicit LogicalCamera(const std::string namespace_ = "");
  ~LogicalCamera();

private:
  void Initialize() override;
  void Deinitialize() override {}

  void PublishData(const void * buffer, int bufferLength);
  void UpdateLogicalCameraData();

  void SetupCameraInfo(zmq::Bridge * const info_bridge_ptr);

private:
  cloisim::msgs::LogicalCameraImage pb_buf_;

  vision_msgs::msg::Detection3DArray msg_;
  sensor_msgs::msg::CameraInfo msg_camera_info_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_LOGICAL_CAMERA__LOGICAL_CAMERA_HPP_
