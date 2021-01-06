/**
 *  @file   CameraDriverSim.cpp
 *  @date   2020-03-20
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include <camera_driver_sim/CameraDriverSim.hpp>
#include <driver_sim/helper.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <protobuf/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace gazebo;

CameraDriverSim::CameraDriverSim(const string node_name)
    : DriverSim(node_name, 2)
{
  topic_name_ = "camera";
  frame_id_ = "camera_link";

  Start();
}

CameraDriverSim::~CameraDriverSim()
{
  // DBG_SIM_INFO("Delete");
  Stop();
}

void CameraDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Data", portData_, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  m_hashKeySub = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  auto pSimBridgeData = GetSimBridge(0);
  auto pSimBridgeInfo = GetSimBridge(1);

  if (pSimBridgeData != nullptr)
  {
    pSimBridgeData->Connect(SimBridge::Mode::SUB, portData_, m_hashKeySub + "Data");
  }

  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, portInfo, m_hashKeySub + "Info");

    GetRos2Parameter(pSimBridgeInfo);

    const auto transform = GetObjectTransform(pSimBridgeInfo);
    SetupStaticTf2(transform, frame_id_ + "_link");

    cameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
    const auto camSensorMsg = GetCameraSensorMessage(pSimBridgeInfo);
    SetCameraInfoInManager(cameraInfoManager, camSensorMsg, frame_id_);
  }

  msg_img.header.frame_id = frame_id_;

  const auto topic_base_name_ = GetPartsName() + "/" + topic_name_;

  image_transport::ImageTransport it(GetNode());
  pubImage = it.advertiseCamera(topic_base_name_ + "/image_raw", 1);
}

void CameraDriverSim::Deinitialize()
{
  pubImage.shutdown();
  DisconnectSimBridges();
}

void CameraDriverSim::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  if (!m_pbImgBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(m_pbImgBuf.time().sec(), m_pbImgBuf.time().nsec());

  msg_img.header.stamp = m_simTime;

  const auto encoding_arg = GetImageEncondingType(m_pbImgBuf.image().pixel_format());
  const uint32_t cols_arg = m_pbImgBuf.image().width();
  const uint32_t rows_arg = m_pbImgBuf.image().height();
  const uint32_t step_arg = m_pbImgBuf.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void *>(m_pbImgBuf.image().data().data()));

  // Publish camera info
  auto camera_info_msg = cameraInfoManager->getCameraInfo();
  camera_info_msg.header.stamp = msg_img.header.stamp;

  pubImage.publish(msg_img, camera_info_msg);
}