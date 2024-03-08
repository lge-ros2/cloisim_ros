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

#ifndef CLOISIM_ROS_CAMERA__SEGMENTATION_CAMERA_HPP_
#define CLOISIM_ROS_CAMERA__SEGMENTATION_CAMERA_HPP_

#include <string>
#include <cloisim_ros_camera/camera_base.hpp>
#include <cloisim_msgs/segmentation.pb.h>
#include <vision_msgs/msg/label_info.hpp>

namespace cloisim_ros
{
class SegmentationCamera : public CameraBase
{
 public:
  explicit SegmentationCamera(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit SegmentationCamera(const std::string node_name = "cloisim_ros_segmentationcamera", const std::string namespace_ = "");
  virtual ~SegmentationCamera();

 private:
  void InitializeCameraData() override;

 private:
  void PublishData(const std::string &buffer);

 private:
  cloisim::msgs::Segmentation pb_seg_;

  //
  // https://github.com/ros-perception/vision_msgs/tree/ros2?tab=readme-ov-file
  //

  // To transmit the metadata associated with the vision pipeline,
  // you should use the /vision_msgs/LabelInfo message.
  // This message works the same as /sensor_msgs/CameraInfo or /vision_msgs/VisionInfo:
  // Publish LabelInfo to a topic.

  // The topic should be at same namespace level as the associated image.
  // That is, if your image is published at /my_segmentation_node/image,
  // the LabelInfo should be published at /my_segmentation_node/label_info.
  // Use a latched publisher for LabelInfo, so that new nodes joining

  // Label info publisher
  rclcpp::Publisher<vision_msgs::msg::LabelInfo>::SharedPtr pub_labelinfo_{nullptr};
};
}  // namespace cloisim_ros

#endif  // CLOISIM_ROS_CAMERA__SEGMENTATION_CAMERA_HPP_