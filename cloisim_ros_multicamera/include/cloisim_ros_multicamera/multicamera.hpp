/**
 *  @file   multicamera.hpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Multi Camera class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef CLOISIM_ROS_MULTICAMERA__MULTICAMERA_HPP_
#define CLOISIM_ROS_MULTICAMERA__MULTICAMERA_HPP_

#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/images_stamped.pb.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cloisim_ros_base/base.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace cloisim_ros
{
class MultiCamera : public Base
{
 public:
  explicit MultiCamera(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit MultiCamera(const std::string namespace_ = "");
  virtual ~MultiCamera();

 private:
  void Initialize() override;
  void Deinitialize() override;

 private:
  void PublishData(const std::string &buffer);

 private:
  // buffer from simulation
  cloisim::msgs::ImagesStamped pb_buf_;

  // message for ROS2 communictaion
  std::map<int, sensor_msgs::msg::Image> msg_imgs_;

  // Camera info managers
  std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> camera_info_manager_;

  // Image publisher
  std::vector<image_transport::CameraPublisher> pubs_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_MULTICAMERA__MULTICAMERA_HPP_
