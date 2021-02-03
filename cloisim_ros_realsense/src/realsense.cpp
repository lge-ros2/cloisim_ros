/**
 *  @file   realsense.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 realsense class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include "cloisim_ros_realsense/realsense.hpp"
#include <cloisim_ros_base/helper.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cloisim_msgs/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace cloisim;
using namespace cloisim_ros;

RealSense::RealSense(const rclcpp::NodeOptions &options_, const string node_name_, const string namespace_)
    : Base(node_name_, namespace_, options_, 9)
    , depth_range_min_(0.1)
    , depth_range_max_(10.0)
    , depth_scale_(1000)
{
  Start(false);
}

RealSense::RealSense(const string namespace_)
    : RealSense(rclcpp::NodeOptions(), "cloisim_ros_realsense", namespace_)
{
}

RealSense::~RealSense()
{
  // DBG_SIM_INFO("Delete");
  Stop();
}

void RealSense::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  int bridgeIndex = 0;
  auto pBridgeInfo = GetBridge(bridgeIndex);

  string header_frame_id = "_camera_link";
  if (pBridgeInfo != nullptr)
  {
    pBridgeInfo->Connect(zmq::Bridge::Mode::CLIENT, portInfo, GetMainHashKey() + "Info");
    GetParameters(pBridgeInfo);
    GetActivatedModules(pBridgeInfo);

    const auto transform = GetObjectTransform(pBridgeInfo);
    header_frame_id = GetPartsName() + header_frame_id;
    SetupStaticTf2(transform, header_frame_id);
  }

  image_transport::ImageTransport it(GetNode());
  for (auto module_name : module_list_)
  {
    uint16_t portCamData, portCamInfo;
    get_parameter_or("bridge." + module_name + "Data", portCamData, uint16_t(0));
    get_parameter_or("bridge." + module_name + "Info", portCamInfo, uint16_t(0));

    const auto topic_base_name_ = GetPartsName() + "/" + module_name;

    const auto hashKeySub = GetMainHashKey() + module_name;

    DBG_SIM_INFO("topic_name:%s, hash Key sub: %s", topic_base_name_.c_str(), hashKeySub.c_str());

    auto pBridgeCamInfo = GetBridge(++bridgeIndex);
    auto pBridgeCamData = GetBridge(++bridgeIndex);
    dataPortMap_[bridgeIndex] = portCamData;
    hashKeySubs_[bridgeIndex] = hashKeySub;

    const auto topic_name = (module_name.find("depth") == string::npos)? "image_raw":"image_rect_raw";
    pubImages_[bridgeIndex] = it.advertiseCamera(topic_base_name_ + "/" + topic_name, 1);

    // handling parameters for image_transport plugin
    const auto format_value = get_parameter("format");
    const auto jpeg_quality_value = get_parameter("jpeg_quality");
    const auto png_level_value = get_parameter("png_level");

    declare_parameters(module_name,
                       map<string, string>{
                           {"format", format_value.as_string()}});
    declare_parameters(module_name,
                       map<string, int>{
                           {"jpeg_quality", jpeg_quality_value.as_int()},
                           {"png_level", png_level_value.as_int()}});

    undeclare_parameter("format");
    undeclare_parameter("jpeg_quality");
    undeclare_parameter("png_level");

    sensor_msgs::msg::Image msg_img;
    msg_img.header.frame_id = module_name;

    msgImgs_[bridgeIndex] = msg_img;

    if (pBridgeCamData != nullptr)
    {
      pBridgeCamData->Connect(zmq::Bridge::Mode::SUB, portCamData, hashKeySub + "Data");
    }

    if (pBridgeCamInfo != nullptr)
    {
      pBridgeCamInfo->Connect(zmq::Bridge::Mode::CLIENT, portCamInfo, hashKeySub + "Info");
      const auto transform = GetObjectTransform(pBridgeCamInfo, module_name);
      const auto child_frame_id = GetPartsName() + "_camera_" + module_name + "_frame";
      SetupStaticTf2(transform, child_frame_id, header_frame_id);

      cameraInfoManager_[bridgeIndex] = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());
      const auto camSensorMsg = GetCameraSensorMessage(pBridgeCamInfo);
      SetCameraInfoInManager(cameraInfoManager_[bridgeIndex], camSensorMsg, module_name);
    }

    threads_.emplace_back(thread([=]() {
      while (IsRunThread())
      {
        UpdateData(bridgeIndex);
      }
    }));
  }
}

void RealSense::Deinitialize()
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
}

void RealSense::GetParameters(zmq::Bridge* const pBridge)
{
  if (pBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  request_msg.set_name("request_realsense_parameters");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get activated module info, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param pbParam;
    if (pbParam.ParseFromString(reply))
    {
      if (pbParam.IsInitialized() &&
          pbParam.name() == "parameters")
      {
        for (auto i = 0; i < pbParam.children_size(); i++)
        {
          auto param = pbParam.children(i);
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

void RealSense::GetActivatedModules(zmq::Bridge* const pBridge)
{
  if (pBridge == nullptr)
  {
    return;
  }

  string moduleListStr;

  msgs::Param request_msg;
  string serializedBuffer;
  request_msg.set_name("request_module_list");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get activated module info, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param pbParam;
    if (pbParam.ParseFromString(reply))
    {
      if (pbParam.IsInitialized() &&
          pbParam.name() == "activated_modules")
      {
        for (auto i = 0; i < pbParam.children_size(); i++)
        {
          auto param = pbParam.children(i);
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

void RealSense::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  cloisim::msgs::ImageStamped pbBuf;
  if (!pbBuf.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBuf.time().sec(), pbBuf.time().nsec());

  auto const msg_img = &msgImgs_[bridge_index];

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
    for (ulong i = 0; i < msg_img->data.size(); i += sizeof(short))
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
    for (ulong i = 0; i < msg_img->data.size(); i += sizeof(short))
    {
      const auto temp = tempImageData[i + 1];
      tempImageData[i + 1] = tempImageData[i];
      tempImageData[i] = temp;
    }
  }
  else if (encoding_arg.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)
  {
    // convert to little-endian
    for (ulong i = 0; i < msg_img->data.size(); i += sizeof(float))
    {
      const auto temp3 = tempImageData[i + 3];
      const auto temp2 = tempImageData[i + 2];
      tempImageData[i + 3] = tempImageData[i];
      tempImageData[i + 2] = tempImageData[i + 1];
      tempImageData[i + 1] = temp2;
      tempImageData[i] = temp3;
    }
  }

  // Publish camera info
  auto camera_info_msg = cameraInfoManager_[bridge_index]->getCameraInfo();
  camera_info_msg.header.stamp = m_simTime;

  pubImages_[bridge_index].publish(*msg_img, camera_info_msg);
}
