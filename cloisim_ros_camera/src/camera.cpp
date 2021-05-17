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

Camera::Camera(const rclcpp::NodeOptions &options_, const string node_name_, const string namespace_)
    : Base(node_name_, namespace_, options_)
{
  topic_name_ = "camera";
  Start();
}

Camera::Camera(const string node_name_, const string namespace_)
    : Camera(rclcpp::NodeOptions(), node_name_, namespace_)
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

  auto pBridgeData = CreateBridge(hashKeyData);
  auto pBridgeInfo = CreateBridge(hashKeyInfo);

  const auto frame_id = GetFrameId("camera_link");
  if (pBridgeInfo != nullptr)
  {
    pBridgeInfo->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(pBridgeInfo);

    const auto transform = GetObjectTransform(pBridgeInfo);
    SetupStaticTf2(transform, frame_id + "_link");

    cameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
    const auto camSensorMsg = GetCameraSensorMessage(pBridgeInfo);
    SetCameraInfoInManager(cameraInfoManager, camSensorMsg, frame_id);
  }

  msgImg.header.frame_id = frame_id;
  const auto topic_base_name_ = GetPartsName() + "/" + topic_name_;

  image_transport::ImageTransport it(GetNode());
  pub = it.advertiseCamera(topic_base_name_ + "/image_raw", 1);

  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    CreatePublisherThread(pBridgeData);
  }
}

void Camera::Deinitialize()
{
  pub.shutdown();
}

void Camera::UpdatePublishingData(const string &buffer)
{
  if (!pbImg.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetSimTime(pbImg.time());

  msgImg.header.stamp = GetSimTime();

  const auto encoding_arg = GetImageEncondingType(pbImg.image().pixel_format());
  const uint32_t cols_arg = pbImg.image().width();
  const uint32_t rows_arg = pbImg.image().height();
  const uint32_t step_arg = pbImg.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(msgImg, encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void *>(pbImg.image().data().data()));

  // Publish camera info
  auto camera_info_msg = cameraInfoManager->getCameraInfo();
  camera_info_msg.header.stamp = msgImg.header.stamp;

  pub.publish(msgImg, camera_info_msg);
}