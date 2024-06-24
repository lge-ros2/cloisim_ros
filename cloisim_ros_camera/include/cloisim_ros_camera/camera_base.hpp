/**
 *  @file   camera_base.hpp
 *  @date   2024-03-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CameraBase class
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef CLOISIM_ROS_CAMERA__CAMERA_BASE_HPP_
#define CLOISIM_ROS_CAMERA__CAMERA_BASE_HPP_

#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/image_stamped.pb.h>
#include <cloisim_msgs/pose.pb.h>

#include <memory>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cloisim_ros_base/base.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace cloisim_ros
{
class CameraBase : public Base
{
 public:
  explicit CameraBase(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit CameraBase(const std::string node_name, const std::string namespace_ = "");
  virtual ~CameraBase();

 protected:
  void Initialize() override;
  void Deinitialize() override;

  void InitializeCameraInfo();

  void InitializeCameraPublish();
  virtual void InitializeCameraData();

 protected:
  void PublishData(const std::string &buffer);
  void PublishData(const cloisim::msgs::ImageStamped& pb_msg);

 protected:
  std::string frame_id_;
  std::string optical_frame_id_;
  std::string topic_base_name_;

  // image buffer from simulator
  cloisim::msgs::ImageStamped pb_img_;

  // message for ROS2 communictaion
  sensor_msgs::msg::Image msg_img_;

  // Image publisher
  image_transport::CameraPublisher pub_;

  // Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_CAMERA__CAMERA_BASE_HPP_
