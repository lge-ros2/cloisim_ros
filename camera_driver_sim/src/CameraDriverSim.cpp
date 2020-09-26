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

CameraDriverSim::CameraDriverSim(const string name)
    : DriverSim(name, 2)
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
  DBG_SIM_INFO("[CONFIG] hash Key sub: %s", m_hashKeySub.c_str());

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
    SetupStaticTf2Message(transform, frame_id_);

    cameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
    const auto camSensorMsg = GetCameraSensorMessage(pSimBridgeInfo);
    SetCameraInfoInManager(cameraInfoManager, camSensorMsg, frame_id_);
  }

  msg_img.header.frame_id = frame_id_;

  const auto topic_base_name_ = GetPartsName() + "/" + topic_name_;

  image_transport::ImageTransport it(GetNode());
  pubImage = it.advertise(topic_base_name_ + "/image_raw", 1);
  pubCameraInfo = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", 1);
}

void CameraDriverSim::Deinitialize()
{
  pubImage.shutdown();
  DisconnectSimBridges();
}

void CameraDriverSim::SetupStaticTf2Message(const gazebo::msgs::Pose transform, const string frame_id)
{
  geometry_msgs::msg::TransformStamped camera_tf;
  camera_tf.header.frame_id = "base_link";
  camera_tf.child_frame_id = frame_id;
  camera_tf.transform.translation.x = transform.position().x();
  camera_tf.transform.translation.y = transform.position().y();
  camera_tf.transform.translation.z = transform.position().z();
  camera_tf.transform.rotation.x = transform.orientation().x();
  camera_tf.transform.rotation.y = transform.orientation().y();
  camera_tf.transform.rotation.z = transform.orientation().z();
  camera_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(camera_tf);
}

void CameraDriverSim::UpdateData(const uint bridge_index)
{
  auto simBridge = GetSimBridge(bridge_index);
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = simBridge->Receive(&pBuffer, bufferLength, false);
  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

    // try reconnect1ion
    if (IsRunThread())
    {
      simBridge->Reconnect(SimBridge::Mode::SUB, portData_, m_hashKeySub);
    }

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

  pubImage.publish(msg_img);

  // Publish camera info
  auto camera_info_msg = cameraInfoManager->getCameraInfo();
  camera_info_msg.header.stamp = m_simTime;

  pubCameraInfo->publish(camera_info_msg);
}