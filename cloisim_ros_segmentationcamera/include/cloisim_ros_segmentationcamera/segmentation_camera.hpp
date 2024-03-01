/**
 *  @file   segmentation_camera.hpp
 *  @date   2024-03-01
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Segmentation Camera class for simulator
 *  @remark
 *  @warning
 *      LGE Advanced Robotics Laboratory
 *      Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_SEGMENTATIONCAMERA__SEGMENTATION_CAMERA_HPP_
#define CLOISIM_ROS_SEGMENTATIONCAMERA__SEGMENTATION_CAMERA_HPP_

#include <string>

#include <cloisim_ros_camera/camera.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cloisim_ros
{
class SegmentationCamera : public Camera
{
 public:
  explicit SegmentationCamera(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit SegmentationCamera(const std::string node_name = "cloisim_ros_segmentationcamera", const std::string namespace_ = "");
  virtual ~SegmentationCamera();

 private:
  void Initialize() override;
  void Deinitialize() override;

 private:
  void PublishData(const std::string &buffer);

 private:
  // Camera info publisher
  // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubDepthCameraInfo{nullptr};

  // Store current point cloud.
  // sensor_msgs::msg::PointCloud2 msg_pc2;

  // Point cloud publisher.
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPointCloud;
};
}  // namespace cloisim_ros

#endif  // CLOISIM_ROS_SEGMENTATIONCAMERA__SEGMENTATION_CAMERA_HPP_