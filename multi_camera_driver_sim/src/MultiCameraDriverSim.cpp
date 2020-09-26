/**
 *  @file   MultiCameraDriverSim.cpp
 *  @date   2020-05-20
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Multi Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include "multi_camera_driver_sim/MultiCameraDriverSim.hpp"
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <driver_sim/helper.h>
#include <protobuf/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace gazebo;

MultiCameraDriverSim::MultiCameraDriverSim()
    : DriverSim("multi_camera_driver_sim", 2)
{
  Start();
}

MultiCameraDriverSim::~MultiCameraDriverSim()
{
  Stop();
}

void MultiCameraDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Data", portData_, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  multicamera_name_ = get_name();

  tf2::Quaternion fixed_rot;
  m_hashKeySub = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  auto pSimBridgeInfo = GetSimBridge(0);
  auto pSimBridgeData = GetSimBridge(1);

  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, portInfo, m_hashKeySub + "Info");

    GetRos2FramesId(pSimBridgeInfo);
  }

  image_transport::ImageTransport it(GetNode());
  for (auto frame_id : frame_id_)
  {
    const auto transform = GetObjectTransform(pSimBridgeInfo, frame_id);
    SetupStaticTf2Message(transform, multicamera_name_ + "_" + frame_id);

    // Image publisher
    const auto topic_base_name_ = multicamera_name_ + "/" + frame_id;
    DBG_SIM_INFO("[CONFIG] topic_base_name:%s", topic_base_name_.c_str());

    pubImages.push_back(it.advertise(topic_base_name_ + "/image_raw", 1));

    // Camera info publisher
    auto camInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", 1);
    pubCamerasInfo.push_back(camInfoPub);

    cameraInfoManager.push_back(std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get()));
    const auto camSensorMsg = GetCameraSensorMessage(pSimBridgeInfo, frame_id);
    SetCameraInfoInManager(cameraInfoManager.back(), camSensorMsg, frame_id);
  }

  pSimBridgeData->Connect(SimBridge::Mode::SUB, portData_, m_hashKeySub + "Data");
}

void MultiCameraDriverSim::Deinitialize()
{
  for (auto pub : pubImages)
  {
    pub.shutdown();
  }

  DisconnectSimBridges();
}

void MultiCameraDriverSim::GetRos2FramesId(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  void *pBuffer = nullptr;
  int bufferLength = 0;

  request_msg.set_name("request_ros2");
  request_msg.SerializeToString(&serializedBuffer);

  pSimBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  const auto succeeded = pSimBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get ROS2 common info, length(%d)", bufferLength);
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }

    if (m_pbBufParam.IsInitialized() &&
        m_pbBufParam.name() == "ros2")
    {
      auto baseParam = m_pbBufParam.children(0);
      if (baseParam.name() == "frames_id")
      {
        for (auto i = 0; i < baseParam.children_size(); i++)
        {
          auto param = baseParam.children(i);
          if (param.name() == "frame_id" && param.has_value())
          {
            const auto frame_id = param.value().string_value();
            frame_id_.push_back(frame_id);
            DBG_SIM_INFO("[CONFIG] frame_id: %s", frame_id.c_str());
          }
        }
      }
    }
  }
}

void MultiCameraDriverSim::UpdateData(const uint bridge_index)
{
  (void)bridge_index;
  auto simBridge = GetSimBridge(1);
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

  if (!m_pbBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(m_pbBuf.time().sec(), m_pbBuf.time().nsec());

  for (auto i = 0; i < m_pbBuf.image_size(); i++)
  {
    auto img = &m_pbBuf.image(i);

    const auto encoding_arg = GetImageEncondingType(img->pixel_format());
    const uint32_t cols_arg = img->width();
    const uint32_t rows_arg = img->height();
    const uint32_t step_arg = img->step();

    msg_img.header.stamp = m_simTime;
    sensor_msgs::fillImage(msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                           reinterpret_cast<const void *>(img->data().data()));

    pubImages.at(i).publish(msg_img);

    // Publish camera info
    auto camera_info_msg = cameraInfoManager[i]->getCameraInfo();
    camera_info_msg.header.stamp = m_simTime;

    pubCamerasInfo[i]->publish(camera_info_msg);
  }
}