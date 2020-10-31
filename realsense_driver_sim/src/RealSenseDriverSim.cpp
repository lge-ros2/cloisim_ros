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
    , depth_range_min_(0.1)
    , depth_range_max_(10.0)
    , depth_scale_(1000)
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

  for (auto module : module_list_)
  {
    uint16_t portCamData, portCamInfo;
    get_parameter_or("bridge." + module + "Data", portCamData, uint16_t(0));
    get_parameter_or("bridge." + module + "Info", portCamInfo, uint16_t(0));

    const auto base_topic_name = GetPartsName() + "/camera/" + module;

    const auto hashKeySub = GetMainHashKey() + module;

    DBG_SIM_INFO("[CONFIG] base_topic_name:%s", base_topic_name.c_str());
    DBG_SIM_INFO("[CONFIG] hash Key sub: %s", hashKeySub.c_str());

    auto pSimBridgeCamInfo = GetSimBridge(++simBridgeCount);
    auto pSimBridgeCamData = GetSimBridge(++simBridgeCount);
    dataPortMap_[simBridgeCount] = portCamData;
    hashKeySubs_[simBridgeCount] = hashKeySub;

    const auto topic_name = (module.find("depth") == string::npos)? "image_raw":"image_rect_raw";

    auto pubImage = image_transport::create_publisher(GetNode(), base_topic_name + "/" + topic_name, rclcpp::QoS(1).get_rmw_qos_profile());
    pubImages_[simBridgeCount] = pubImage;

    auto pubCameraInfo = create_publisher<sensor_msgs::msg::CameraInfo>(base_topic_name + "/camera_info", rclcpp::QoS(1));
    pubCameraInfos_[simBridgeCount] = pubCameraInfo;

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

      GetCameraSensorMessage(pSimBridgeCamInfo);
      InitializeCameraInfoMessage(simBridgeCount, module);
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

void RealSenseDriverSim::GetParameters(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  string serializedBuffer;
  void *pBuffer = nullptr;
  int bufferLength = 0;

  request_msg.set_name("request_realsense_parameters");
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
        m_pbBufParam.name() == "parameters")
    {
      for (auto i = 0; i < m_pbBufParam.children_size(); i++)
      {
        auto param = m_pbBufParam.children(i);
        if (param.name() == "depth_range_min" && param.has_value())
        {
          depth_range_min_ = param.value().double_value();
          DBG_SIM_INFO("[CONFIG] depth_range_min: %f", depth_range_min_);
        }
        else if (param.name() == "depth_range_max" && param.has_value())
        {
          depth_range_max_ = param.value().double_value();
          DBG_SIM_INFO("[CONFIG] depth_range_max: %f", depth_range_max_);
        }
        else if (param.name() == "depth_scale" && param.has_value())
        {
          depth_scale_ = param.value().int_value();
          DBG_SIM_INFO("[CONFIG] depth_scale: %d", depth_scale_);
        }
        else
        {
          DBG_SIM_WRN("[CONFIG] Wrong params %s", param.name().c_str());
        }
      }
    }
  }
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
        if (param.name() == "module" && param.has_value() &&
            param.value().type() == msgs::Any_ValueType_STRING &&
            !param.value().string_value().empty())
        {
          const auto module = param.value().string_value();
          module_list_.push_back(module);
          DBG_SIM_INFO("[CONFIG] activated_module: %s", module.c_str());
        }
      }
    }
  }
}

void RealSenseDriverSim::GetCameraSensorMessage(SimBridge* const pSimBridge)
{
  if (pSimBridge == nullptr)
  {
    return;
  }

  msgs::Param request_msg;
  request_msg.set_name("request_camera_info");

  string serializedBuffer;
  request_msg.SerializeToString(&serializedBuffer);

  pSimBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  void *pBuffer = nullptr;
  int bufferLength = 0;
  const auto succeeded = pSimBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get camera info, length(%d)", bufferLength);
  }
  else
  {
    if (m_pbTmpBufCameraSensorInfo.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }
  }
}

void RealSenseDriverSim::InitializeCameraInfoMessage(const uint bridge_index, const string frame_id)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;

  int width_ = m_pbTmpBufCameraSensorInfo.image_size().x();
  int height_ = m_pbTmpBufCameraSensorInfo.image_size().y();

  // C parameters
  auto cx_ = (static_cast<double>(width_) + 1.0) / 2.0;
  auto cy_ = (static_cast<double>(height_) + 1.0) / 2.0;

  double hfov_ = m_pbTmpBufCameraSensorInfo.horizontal_fov();

  auto computed_focal_length = (static_cast<double>(width_)) / (2.0 * tan(hfov_ / 2.0));

  // CameraInfo
  camera_info_msg.header.frame_id = frame_id;
  camera_info_msg.height = height_;
  camera_info_msg.width = width_;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  const auto hack_baseline = 0.0f;

  // Get distortion from camera
  double distortion_k1 = m_pbTmpBufCameraSensorInfo.distortion().k1();
  double distortion_k2 = m_pbTmpBufCameraSensorInfo.distortion().k2();
  double distortion_k3 = m_pbTmpBufCameraSensorInfo.distortion().k3();
  double distortion_t1 = m_pbTmpBufCameraSensorInfo.distortion().p1();
  double distortion_t2 = m_pbTmpBufCameraSensorInfo.distortion().p2();

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg.d[0] = distortion_k1;
  camera_info_msg.d[1] = distortion_k2;
  camera_info_msg.d[2] = distortion_t1;
  camera_info_msg.d[3] = distortion_t2;
  camera_info_msg.d[4] = distortion_k3;

  // Original camera matrix
  camera_info_msg.k.fill(0.0);
  camera_info_msg.k[0] = computed_focal_length;
  camera_info_msg.k[2] = cx_;
  camera_info_msg.k[4] = computed_focal_length;
  camera_info_msg.k[5] = cy_;

  // rectification
  camera_info_msg.r.fill(0.0);
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[8] = 1.0;

  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.p.fill(0.0);
  camera_info_msg.p[0] = computed_focal_length;
  camera_info_msg.p[2] = cx_;
  camera_info_msg.p[3] = -computed_focal_length * hack_baseline;
  camera_info_msg.p[5] = computed_focal_length;
  camera_info_msg.p[6] = cy_;
  camera_info_msg.p[10] = 1.0;

  // Initialize camera_info_manager
  cameraInfoManager_[bridge_index] = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode(), frame_id);
  cameraInfoManager_[bridge_index]->setCameraInfo(camera_info_msg);
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