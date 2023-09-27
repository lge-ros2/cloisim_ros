/**
 *  @file   depthcamera.hpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Depth Camera class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#ifndef CLOISIM_ROS_DEPTHCAMERA__DEPTHCAMERA_HPP_
#define CLOISIM_ROS_DEPTHCAMERA__DEPTHCAMERA_HPP_

#include <string>

#include <cloisim_ros_camera/camera.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cloisim_ros
{
class DepthCamera : public Camera
{
 public:
  explicit DepthCamera(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
  explicit DepthCamera(const std::string node_name = "cloisim_ros_depthcamera", const std::string namespace_ = "");
  virtual ~DepthCamera();

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
#endif  // CLOISIM_ROS_DEPTHCAMERA__DEPTHCAMERA_HPP_
