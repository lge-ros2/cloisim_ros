/**
 *  @file   camera.hpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Camera class for cloisim
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef CLOISIM_ROS_CAMERA__CAMERA_HPP_
#define CLOISIM_ROS_CAMERA__CAMERA_HPP_

#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/image_stamped.pb.h>
#include <cloisim_msgs/pose.pb.h>

#include <memory>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cloisim_ros_camera/camera_base.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace cloisim_ros
{
class Camera : public CameraBase
{
 public:
  explicit Camera(
    const rclcpp::NodeOptions &options_,
    const std::string node_name = "cloisim_ros_camera", const std::string namespace_ = "");
  explicit Camera(
    const std::string node_name = "cloisim_ros_camera", const std::string namespace_ = "");
  virtual ~Camera();
};
}  // namespace cloisim_ros

#endif  // CLOISIM_ROS_CAMERA__CAMERA_HPP_
