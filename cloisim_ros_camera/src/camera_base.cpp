/**
 *  @file   camera_base.cpp
 *  @date   2024-03-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CameraBase class
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <cloisim_msgs/param.pb.h>

#include <cstring>
#include <cloisim_ros_base/camera_helper.hpp>
#include <cloisim_ros_camera/camera.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;
using namespace std::literals::chrono_literals;
using std::string;

namespace cloisim_ros
{
CameraBase::CameraBase(
  const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: Base(node_name, namespace_, options_)
  , frame_id_("camera_link")
  , optical_frame_id_("camera_optical_frame")
  , topic_base_name_("")
{
  topic_name_ = "camera";
  // DBG_SIM_INFO("CameraBase");
}

CameraBase::CameraBase(const string node_name, const string namespace_)
: CameraBase(rclcpp::NodeOptions(), node_name, namespace_)
{
  // DBG_SIM_INFO("CameraBase");
}

CameraBase::~CameraBase()
{
  // DBG_SIM_INFO("Delete CameraBase");
}

void CameraBase::Initialize()
{
  // DBG_SIM_INFO("CameraBase Initialization");
  InitializeCameraInfo();
  InitializeCameraPublish();
  InitializeCameraData();
}

void CameraBase::InitializeCameraInfo()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyInfo = GetTargetHashKey("Info");

  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    frame_id_ = GetPartsName() + "_" + GetFrameId("camera_link");
    auto parent_frame_id = std::string("base_link");
    auto link_frame_transform_pose = GetObjectTransform(info_bridge_ptr, parent_frame_id);
    link_frame_transform_pose.set_name(frame_id_);
    SetStaticTf2(link_frame_transform_pose, parent_frame_id);

    optical_frame_id_ = GetPartsName() + "_camera_optical_frame";
    cloisim::msgs::Pose optical_frame_transform_pose;
    optical_frame_transform_pose.mutable_orientation()->set_x(-0.5);
    optical_frame_transform_pose.mutable_orientation()->set_y(0.5);
    optical_frame_transform_pose.mutable_orientation()->set_z(-0.5);
    optical_frame_transform_pose.mutable_orientation()->set_w(0.5);
    optical_frame_transform_pose.set_name(optical_frame_id_);
    SetStaticTf2(optical_frame_transform_pose, frame_id_);

    camera_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
    const auto camSensorMsg = GetCameraSensorMessage(info_bridge_ptr);
    SetCameraInfoInManager(camera_info_manager_, camSensorMsg, frame_id_);
  }

  LOG_I(this, "hashKey: info(" << hashKeyInfo << ")");
}

void CameraBase::InitializeCameraPublish()
{
  msg_img_.header.frame_id = optical_frame_id_;

  topic_base_name_ = GetPartsName() + "/" + topic_name_;

  image_transport::ImageTransport it(GetNode());
  pub_ = it.advertiseCamera(topic_base_name_ + "/image_raw", 1);
}

void CameraBase::InitializeCameraData()
{
  uint16_t portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");

  auto data_bridge_ptr = CreateBridge();

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(
      data_bridge_ptr,
      bind(
        static_cast<void (CameraBase::*)(const void *, int)>(&CameraBase::PublishData), this,
        std::placeholders::_1, std::placeholders::_2));
  }

  LOG_I(this, "hashKey: data(" << hashKeyData << ")");
}

void CameraBase::Deinitialize() {pub_.shutdown();}

void CameraBase::PublishData(const void * buffer, int bufferLength)
{
  if (!pb_img_.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "##Parsing error, size=" << bufferLength);
    return;
  }

  PublishData(pb_img_);
}

void CameraBase::PublishData(const cloisim::msgs::ImageStamped & pb_msg)
{
  SetTime(pb_msg.time());

  msg_img_.header.stamp = GetTime();

  const auto & image = pb_msg.image();
  msg_img_.encoding = GetImageEncondingType(image.pixel_format());
  msg_img_.width = image.width();
  msg_img_.height = image.height();
  msg_img_.step = image.step();
  msg_img_.is_bigendian = false;

  // Direct copy: resize only if needed, then memcpy (avoids fillImage overhead)
  const auto & src_data = image.data();
  const auto data_size = src_data.size();
  if (msg_img_.data.size() != data_size) {
    msg_img_.data.resize(data_size);
  }
  std::memcpy(msg_img_.data.data(), src_data.data(), data_size);

  // Publish camera info
  auto camera_info_msg = camera_info_manager_->getCameraInfo();
  camera_info_msg.header.stamp = msg_img_.header.stamp;

  pub_.publish(msg_img_, camera_info_msg);
}
}  // namespace cloisim_ros
