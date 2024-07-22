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

using string = std::string;

namespace cloisim_ros
{

DepthCamera::DepthCamera(
    const rclcpp::NodeOptions &options_,
    const string node_name,
    const string namespace_)
    : CameraBase(options_, node_name, namespace_)
{
  Start();
}

DepthCamera::DepthCamera(
    const string node_name,
    const string namespace_)
    : DepthCamera(rclcpp::NodeOptions(), node_name, namespace_)
{
}

DepthCamera::~DepthCamera()
{
  // DBG_SIM_INFO("Delete DepthCamera");
  Stop();
}

}  // namespace cloisim_ros
