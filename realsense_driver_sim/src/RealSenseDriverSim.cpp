/**
 *  @file   RealSenseDriverSim.cpp
 *  @date   2020-03-20
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Depth Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include "realsense_driver_sim/RealSenseDriverSim.hpp"
#include <driver_sim/helper.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <protobuf/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace gazebo;

RealSenseDriverSim::RealSenseDriverSim()
    : DriverSim("realsense_driver_sim", 9)
{
  Start(false);
}

RealSenseDriverSim::~RealSenseDriverSim()
{
  DBG_SIM_INFO("Delete");
  Stop();
}

void RealSenseDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  int simBridgeCount = 0;
  auto pSimBridgeInfo = GetSimBridge(simBridgeCount);

  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, portInfo, GetMainHashKey() + "Info");
    GetActivatedModules(pSimBridgeInfo);
  }

  image_transport::ImageTransport it(GetNode());

  for (auto module : module_list_)
  {
    uint16_t portCamData, portCamInfo;
    get_parameter_or("bridge." + module + "Data", portCamData, uint16_t(0));
    get_parameter_or("bridge." + module + "Info", portCamInfo, uint16_t(0));

    const auto topic_base_name_ = GetPartsName() + "/" + module;

    const auto hashKeySub = GetMainHashKey() + module;

    DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_base_name_.c_str());
    DBG_SIM_INFO("[CONFIG] hash Key sub: %s", hashKeySub.c_str());

    auto pSimBridgeCamInfo = GetSimBridge(++simBridgeCount);
    auto pSimBridgeCamData = GetSimBridge(++simBridgeCount);
    dataPortMap_[simBridgeCount] = portCamData;
    hashKeySubs_[simBridgeCount] = hashKeySub;

    pubImages_[simBridgeCount] = it.advertise(topic_base_name_ + "/image_raw", 1);

    // TODO: to supress the error log
    // -> Failed to load plugin image_transport/compressed_pub, error string: parameter 'format' has already been declared
    // -> Failed to load plugin image_transport/compressed_pub, error string: parameter 'png_level' has already been declared
    // -> Failed to load plugin image_transport/compressed_pub, error string: parameter 'jpeg_quality' has already been declared
    undeclare_parameter("format");
    undeclare_parameter("png_level");
    undeclare_parameter("jpeg_quality");

    pubCameraInfos_[simBridgeCount] = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", 1);;

    sensor_msgs::msg::Image msg_img;
    msg_img.header.frame_id = module;

    msg_imgs_[simBridgeCount] = msg_img;

    if (pSimBridgeCamData != nullptr)
    {
      pSimBridgeCamData->Connect(SimBridge::Mode::SUB, portCamData, hashKeySub + "Data");
    }

    if (pSimBridgeCamInfo != nullptr)
    {
      pSimBridgeCamInfo->Connect(SimBridge::Mode::CLIENT, portCamInfo, hashKeySub + "Info");
      const auto transform = GetObjectTransform(pSimBridgeCamInfo, module);
      SetupStaticTf2Message(transform, module);

      cameraInfoManager_[simBridgeCount] = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
      const auto camSensorMsg = GetCameraSensorMessage(pSimBridgeCamInfo);
      SetCameraInfoInManager(cameraInfoManager_[simBridgeCount], camSensorMsg, module);
    }

    threads_.emplace_back(thread([=]() {
      while (IsRunThread())
      {
        UpdateData(simBridgeCount);
      }
    }));
  }
}

void RealSenseDriverSim::Deinitialize()
{
  for (auto &thread : threads_)
  {
    if (thread.joinable())
    {
      thread.join();
      DBG_SIM_INFO("Thread finished");
    }
  }

  for (auto pub : pubImages_)
  {
    pub.second.shutdown();
  }

  DisconnectSimBridges();
}

void RealSenseDriverSim::GetActivatedModules(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  void *pBuffer = nullptr;
  int bufferLength = 0;

  request_msg.set_name("request_module_list");
  request_msg.SerializeToString(&serializedBuffer);

  pSimBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  const auto succeeded = pSimBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get activated module info, length(%d)", bufferLength);
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }

    if (m_pbBufParam.IsInitialized() &&
        m_pbBufParam.name() == "activated_modules")
    {
      for (auto i = 0; i < m_pbBufParam.children_size(); i++)
      {
        auto param = m_pbBufParam.children(i);
        if (param.name() == "module" && param.has_value())
        {
          const auto module = param.value().string_value();
          module_list_.push_back(module);
          DBG_SIM_INFO("[CONFIG] activated_module: %s", module.c_str());
        }
      }
    }
  }
}

void RealSenseDriverSim::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  auto simBridge = GetSimBridge(bridge_index);
  if (simBridge == nullptr)
  {
    DBG_SIM_ERR("sim bridge is null!!");
    return;
  }

  const bool succeeded = simBridge->Receive(&pBuffer, bufferLength, false);
  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

    // try reconnect1ion
    if (IsRunThread())
    {
      simBridge->Reconnect(SimBridge::Mode::SUB, dataPortMap_[bridge_index], hashKeySubs_[bridge_index]);
    }

    return;
  }

  gazebo::msgs::ImageStamped pbBuf;
  if (!pbBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBuf.time().sec(), pbBuf.time().nsec());

  msg_imgs_[bridge_index].header.stamp = m_simTime;

  const auto encoding_arg = GetImageEncondingType(pbBuf.image().pixel_format());
  const uint32_t cols_arg = pbBuf.image().width();
  const uint32_t rows_arg = pbBuf.image().height();
  const uint32_t step_arg = pbBuf.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(msg_imgs_[bridge_index], encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void *>(pbBuf.image().data().data()));

  pubImages_[bridge_index].publish(msg_imgs_[bridge_index]);

  // Publish camera info
  auto camera_info_msg = cameraInfoManager_[bridge_index]->getCameraInfo();
  camera_info_msg.header.stamp = m_simTime;

  pubCameraInfos_[bridge_index]->publish(camera_info_msg);
}