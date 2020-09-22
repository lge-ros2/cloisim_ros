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

  for (auto frame_id : frame_id_)
  {
    const auto transform = GetObjectTransform(pSimBridgeInfo, frame_id);
    SetupStaticTf2Message(transform, frame_id);

    // Image publisher
    const auto topic_base_name_ = multicamera_name_ + "/" + frame_id;
    DBG_SIM_INFO("[CONFIG] topic_base_name:%s", topic_base_name_.c_str());

    pubImages.push_back(image_transport::create_publisher(GetNode(), topic_base_name_ + "/image_raw", rclcpp::QoS(1).get_rmw_qos_profile()));

    // Camera info publisher
    auto camInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", rclcpp::QoS(1));
    pubCamerasInfo.push_back(camInfoPub);

    GetCameraSensorMessage(pSimBridgeInfo, frame_id);
    InitializeCameraInfoMessage(frame_id);
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

void MultiCameraDriverSim::SetupStaticTf2Message(const gazebo::msgs::Pose transform, const string frame_id)
{
  geometry_msgs::msg::TransformStamped camera_tf;
  camera_tf.header.frame_id = "base_link";
  camera_tf.child_frame_id = multicamera_name_ + "_" + frame_id;
  camera_tf.transform.translation.x = transform.position().x();
  camera_tf.transform.translation.y = transform.position().y();
  camera_tf.transform.translation.z = transform.position().z();
  camera_tf.transform.rotation.x = transform.orientation().x();
  camera_tf.transform.rotation.y = transform.orientation().y();
  camera_tf.transform.rotation.z = transform.orientation().z();
  camera_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(camera_tf);
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

void MultiCameraDriverSim::GetCameraSensorMessage(SimBridge* const pSimBridge, const string camera_name)
{
  msgs::Param request_msg;
  request_msg.set_name("request_camera_info");

  auto pVal = request_msg.mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(camera_name);

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

void MultiCameraDriverSim::InitializeCameraInfoMessage(const string frame_id)
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
  auto distortion_k1 = m_pbTmpBufCameraSensorInfo.distortion().k1();
  auto distortion_k2 = m_pbTmpBufCameraSensorInfo.distortion().k2();
  auto distortion_k3 = m_pbTmpBufCameraSensorInfo.distortion().k3();
  auto distortion_t1 = m_pbTmpBufCameraSensorInfo.distortion().p1();
  auto distortion_t2 = m_pbTmpBufCameraSensorInfo.distortion().p2();

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
  cameraInfoManager.push_back(std::make_shared<camera_info_manager::CameraInfoManager>(GetNode(), frame_id));
  cameraInfoManager.back()->setCameraInfo(camera_info_msg);
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