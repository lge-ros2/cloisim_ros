/**
 *  @file   depth_camera.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Depth Camera class for simulator
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include "cloisim_ros_camera/depth_camera.hpp"

namespace cloisim_ros
{

DepthCamera::DepthCamera(
  const rclcpp::NodeOptions & options_, const std::string node_name, const std::string namespace_)
: CameraBase(options_, node_name, namespace_)
{
  // Pre-declare enable_pub_plugins to exclude the "compressed" transport,
  // which does not support depth encodings (16UC1, 32FC1) and spams errors.
  const auto param_name =
    std::string(get_name()) + ".depth.image_raw.enable_pub_plugins";
  this->declare_parameter(
    param_name,
    std::vector<std::string>{
      "image_transport/raw",
      "image_transport/compressedDepth",
      "image_transport/theora",
      "image_transport/zstd"});

  Start();
}

DepthCamera::DepthCamera(const std::string node_name, const std::string namespace_)
: DepthCamera(rclcpp::NodeOptions(), node_name, namespace_)
{
}

DepthCamera::~DepthCamera()
{
  // DBG_SIM_INFO("Delete DepthCamera");
  Stop();
}

}  // namespace cloisim_ros
