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

RealSenseDriverSim::RealSenseDriverSim(const string node_name)
    : DriverSim(node_name, 9)
    , depth_range_min_(0.1)
    , depth_range_max_(10.0)
    , depth_scale_(1000)
{
  Start(false);
}

RealSenseDriverSim::~RealSenseDriverSim()
{
  // DBG_SIM_INFO("Delete");
  Stop();
}

void RealSenseDriverSim::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  int simBridgeCount = 0;
  auto pSimBridgeInfo = GetSimBridge(simBridgeCount);

  string header_frame_id = "_camera_link";
  if (pSimBridgeInfo != nullptr)
  {
    pSimBridgeInfo->Connect(SimBridge::Mode::CLIENT, portInfo, GetMainHashKey() + "Info");
    GetParameters(pSimBridgeInfo);
    GetActivatedModules(pSimBridgeInfo);

    const auto transform = GetObjectTransform(pSimBridgeInfo);
    header_frame_id = GetPartsName() + header_frame_id;
    SetupStaticTf2(transform, header_frame_id);
  }

  image_transport::ImageTransport it(GetNode());

  for (auto module : module_list_)
  {
    uint16_t portCamData, portCamInfo;
    get_parameter_or("bridge." + module + "Data", portCamData, uint16_t(0));
    get_parameter_or("bridge." + module + "Info", portCamInfo, uint16_t(0));

    const auto topic_base_name_ = GetPartsName() + "/" + module;

    const auto hashKeySub = GetMainHashKey() + module;

    DBG_SIM_INFO("topic_name:%s, hash Key sub: %s", topic_base_name_.c_str(), hashKeySub.c_str());

    auto pSimBridgeCamInfo = GetSimBridge(++simBridgeCount);
    auto pSimBridgeCamData = GetSimBridge(++simBridgeCount);
    dataPortMap_[simBridgeCount] = portCamData;
    hashKeySubs_[simBridgeCount] = hashKeySub;

    const auto topic_name = (module.find("depth") == string::npos)? "image_raw":"image_rect_raw";
    pubImages_[simBridgeCount] = it.advertise(topic_base_name_ + "/" + topic_name, 1);

    // TODO: to supress the error log
    // -> Failed to load plugin image_transport/compressed_pub, error string: parameter 'format' has already been declared
    // -> Failed to load plugin image_transport/compressed_pub, error string: parameter 'png_level' has already been declared
    // -> Failed to load plugin image_transport/compressed_pub, error string: parameter 'jpeg_quality' has already been declared
    undeclare_parameter("format");
    undeclare_parameter("png_level");
    undeclare_parameter("jpeg_quality");

    pubCameraInfos_[simBridgeCount] = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", 1);

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
      const auto child_frame_id = GetPartsName() + "_camera_" + module + "_frame";
      SetupStaticTf2(transform, child_frame_id, header_frame_id);

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
      // DBG_SIM_INFO("Thread finished");
    }
  }

  for (auto pub : pubImages_)
  {
    pub.second.shutdown();
  }

  DisconnectSimBridges();
}

void RealSenseDriverSim::GetParameters(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  request_msg.set_name("request_realsense_parameters");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pSimBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get activated module info, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromString(reply))
    {
      if (m_pbBufParam.IsInitialized() &&
          m_pbBufParam.name() == "parameters")
      {
        for (auto i = 0; i < m_pbBufParam.children_size(); i++)
        {
          auto param = m_pbBufParam.children(i);
          if (param.name() == "depth_range_min" && param.has_value())
          {
            depth_range_min_ = param.value().double_value();
          }
          else if (param.name() == "depth_range_max" && param.has_value())
          {
            depth_range_max_ = param.value().double_value();
          }
          else if (param.name() == "depth_scale" && param.has_value())
          {
            depth_scale_ = param.value().int_value();
          }
          else
          {
            DBG_SIM_WRN("Wrong params %s", param.name().c_str());
          }
        }
      }
    }
    else
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%ld)", reply.data(), reply.size());
    }

    DBG_SIM_INFO("depth_range_min: %f, depth_range_max: %f, depth_scale: %d", depth_range_min_, depth_range_max_, depth_scale_);
  }
}

void RealSenseDriverSim::GetActivatedModules(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  string moduleListStr;

  msgs::Param request_msg;
  string serializedBuffer;
  request_msg.set_name("request_module_list");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pSimBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get activated module info, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromString(reply))
    {
      if (m_pbBufParam.IsInitialized() &&
          m_pbBufParam.name() == "activated_modules")
      {
        for (auto i = 0; i < m_pbBufParam.children_size(); i++)
        {
          auto param = m_pbBufParam.children(i);
        if (param.name() == "module" && param.has_value() &&
            param.value().type() == msgs::Any_ValueType_STRING &&
            !param.value().string_value().empty())
          {
            const auto module = param.value().string_value();
            module_list_.push_back(module);
            moduleListStr.append(module + ", ");
          }
        }
      }
    }
    else
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%ld)", reply.data(), reply.size());
    }
  }

  DBG_SIM_INFO("activated_modules: %s", moduleListStr.c_str());
}

void RealSenseDriverSim::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  gazebo::msgs::ImageStamped pbBuf;
  if (!pbBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBuf.time().sec(), pbBuf.time().nsec());

  auto const msg_img = &msg_imgs_[bridge_index];

  msg_img->header.stamp = m_simTime;

  const auto encoding_arg = GetImageEncondingType(pbBuf.image().pixel_format());
  const uint32_t cols_arg = pbBuf.image().width();
  const uint32_t rows_arg = pbBuf.image().height();
  const uint32_t step_arg = pbBuf.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(*msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void *>(pbBuf.image().data().data()));

  // Post processing
  auto tempImageData = msg_img->data.data();

  if (encoding_arg.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0)
  {
    // for only depth sensor(Z16)
    for (uint i = 0; i < msg_img->data.size(); i += 2)
    {
      const auto depthDataInUInt16 = (uint16_t)tempImageData[i] << 8 | (uint16_t)tempImageData[i + 1];
      const auto scaledDepthData = (double)depthDataInUInt16 / (double)UINT16_MAX;
      const auto realDepthRange = (uint16_t)(scaledDepthData * depth_range_max_ * (double)depth_scale_);

      // convert to little-endian
      tempImageData[i + 1] = (uint8_t)(realDepthRange >> 8);
      tempImageData[i] = (uint8_t)(realDepthRange);
    }
  }
  else if (encoding_arg.compare(sensor_msgs::image_encodings::TYPE_16SC1) == 0)
  {
    // convert to little-endian
    for (uint i = 0; i < msg_img->data.size(); i += 2)
    {
      const auto temp = tempImageData[i + 1];
      tempImageData[i + 1] = tempImageData[i];
      tempImageData[i] = temp;
    }
  }
  else if (encoding_arg.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)
  {
    // convert to little-endian
    for (uint i = 0; i < msg_img->data.size(); i += 4)
    {
      const auto temp3 = tempImageData[i + 3];
      const auto temp2 = tempImageData[i + 2];
      tempImageData[i + 3] = tempImageData[i];
      tempImageData[i + 2] = tempImageData[i + 1];
      tempImageData[i + 1] = temp2;
      tempImageData[i] = temp3;
    }
  }

  pubImages_[bridge_index].publish(*msg_img);

  // Publish camera info
  auto camera_info_msg = cameraInfoManager_[bridge_index]->getCameraInfo();
  camera_info_msg.header.stamp = m_simTime;

  pubCameraInfos_[bridge_index]->publish(camera_info_msg);
}
