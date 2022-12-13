/**
 *  @file   camera.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Camera class for cloisim
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include <cloisim_ros_camera/camera.hpp>
#include <cloisim_ros_base/camera_helper.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cloisim_msgs/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace cloisim_ros;

Camera::Camera(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
{
  topic_name_ = "camera";
  Start();
}

Camera::Camera(const string node_name, const string namespace_)
    : Camera(rclcpp::NodeOptions(), node_name, namespace_)
{
}

Camera::~Camera()
{
  // DBG_SIM_INFO("Delete");
  Stop();
}

void Camera::Initialize()
{
  uint16_t portInfo, portData;;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  DBG_SIM_INFO("hash Key: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  string frame_id = "camera_link";
  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    frame_id = GetPartsName() + "_" + GetFrameId("camera_link");
    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);

    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
    const auto camSensorMsg = GetCameraSensorMessage(info_bridge_ptr);
    SetCameraInfoInManager(camera_info_manager_, camSensorMsg, frame_id);
  }

  msg_img_.header.frame_id = frame_id;
  const auto topic_base_name_ = GetPartsName() + "/" + topic_name_;

  image_transport::ImageTransport it(GetNode());
  pub_ = it.advertiseCamera(topic_base_name_ + "/image_raw", 1);

  if (data_bridge_ptr != nullptr)
  {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&Camera::PublishData, this, std::placeholders::_1));
  }
}

void Camera::Deinitialize()
{
  pub_.shutdown();
}

void Camera::PublishData(const string &buffer)
{
  if (!pb_img_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_img_.time());

  msg_img_.header.stamp = GetTime();

  const auto encoding_arg = GetImageEncondingType(pb_img_.image().pixel_format());
  const uint32_t cols_arg = pb_img_.image().width();
  const uint32_t rows_arg = pb_img_.image().height();
  const uint32_t step_arg = pb_img_.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(msg_img_, encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void *>(pb_img_.image().data().data()));

  // Publish camera info
  auto camera_info_msg = camera_info_manager_->getCameraInfo();
  camera_info_msg.header.stamp = msg_img_.header.stamp;

  pub_.publish(msg_img_, camera_info_msg);
}