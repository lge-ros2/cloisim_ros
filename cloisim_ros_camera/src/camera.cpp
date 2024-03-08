/**
 *  @file   camera.cpp
 *  @date   2024-03-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Camera class
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include <cloisim_msgs/param.pb.h>
#include <cloisim_ros_base/camera_helper.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cloisim_ros_camera/camera.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std;
using namespace std::chrono_literals;

namespace cloisim_ros
{
Camera::Camera(
    const rclcpp::NodeOptions &options_,
    const string node_name,
    const string namespace_)
    : CameraBase(options_, node_name, namespace_)
{
  DBG_SIM_INFO("Camera");
  Start();
}

Camera::Camera(
    const string node_name,
    const string namespace_)
    : Camera(rclcpp::NodeOptions(), node_name, namespace_)
{
  // DBG_SIM_INFO("Camera");
}

Camera::~Camera()
{
  // DBG_SIM_INFO("Delete Camera");
  Stop();
}
}  // namespace cloisim_ros
