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
#include "camera_driver_sim/CameraDriverSim.hpp"
#include "driver_sim/helper.h"
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <protobuf/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace gazebo;

CameraDriverSim::CameraDriverSim(const std::string node_name)
    : DriverSim(node_name, 2)
{
  Start();
}

CameraDriverSim::~CameraDriverSim()
{
  DBG_SIM_INFO("Delete");
  Stop();
}

void CameraDriverSim::Initialize()
{
  string frame_id_;
  string topic_name_;
  vector<double> transform_;

  get_parameter_or("topic_name", topic_name_, string("topic"));
  get_parameter_or("frame_id", frame_id_, string("camera_link"));
  get_parameter_or("transform", transform_, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  const auto topic_base_name_ = GetPartsName() + "/" + topic_name_;
  m_hashKeySub = GetRobotName() + GetPartsName();

  DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_base_name_.c_str());
  DBG_SIM_INFO("[CONFIG] hash Key sub: %s", m_hashKeySub.c_str());

  geometry_msgs::msg::TransformStamped camera_tf;
  tf2::Quaternion fixed_rot;
  fixed_rot.setRPY(transform_[3], transform_[4], transform_[5]);

  camera_tf.header.frame_id = "base_footprint";
  camera_tf.child_frame_id = frame_id_;
  camera_tf.transform.translation.x = transform_[0];
  camera_tf.transform.translation.y = transform_[1];
  camera_tf.transform.translation.z = transform_[2];
  camera_tf.transform.rotation.x = fixed_rot.x();
  camera_tf.transform.rotation.y = fixed_rot.y();
  camera_tf.transform.rotation.z = fixed_rot.z();
  camera_tf.transform.rotation.w = fixed_rot.w();

  AddStaticTf2(camera_tf);

  msg_img.header.frame_id = frame_id_;

  GetSimBridge(0)->Connect(SimBridge::Mode::SUB, m_hashKeySub);
  GetSimBridge(1)->Connect(SimBridge::Mode::CLIENT, m_hashKeySub + "Info");

  pubImage = image_transport::create_publisher(GetNode(), topic_base_name_ + "/image_raw");

  pubCameraInfo = create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", rclcpp::SensorDataQoS());

  cameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode(), GetPartsName());

  GetCameraSensorMessage();
  InitializeCameraInfoMessage(frame_id_);
}

void CameraDriverSim::Deinitialize()
{
  pubImage.shutdown();

  GetSimBridge(0)->Disconnect();
  GetSimBridge(1)->Disconnect();
}

void CameraDriverSim::GetCameraSensorMessage()
{
  msgs::Param request_msg;
  request_msg.set_name("request_camera_info");

  string serializedBuffer;
  request_msg.SerializeToString(&serializedBuffer);

  auto simBridge = GetSimBridge(1);
  simBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  void *pBuffer = nullptr;
  int bufferLength = 0;
  const auto succeeded = simBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get camera info, length(%d)", bufferLength);
  }
  else
  {
    if (m_pbBufCameraSensorInfo.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }
  }
}

void CameraDriverSim::InitializeCameraInfoMessage(const string frame_id)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;

  int width_ = m_pbBufCameraSensorInfo.image_size().x();
  int height_ = m_pbBufCameraSensorInfo.image_size().y();

  // C parameters
  auto cx_ = (static_cast<double>(width_) + 1.0) / 2.0;
  auto cy_ = (static_cast<double>(height_) + 1.0) / 2.0;

  double hfov_ = m_pbBufCameraSensorInfo.horizontal_fov();

  auto computed_focal_length = (static_cast<double>(width_)) / (2.0 * tan(hfov_ / 2.0));

  // CameraInfo
  camera_info_msg.header.frame_id = frame_id;
  camera_info_msg.height = height_;
  camera_info_msg.width = width_;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  const auto hack_baseline = 0.0f;

  // Get distortion from camera
  double distortion_k1 = m_pbBufCameraSensorInfo.distortion().k1();
  double distortion_k2 = m_pbBufCameraSensorInfo.distortion().k2();
  double distortion_k3 = m_pbBufCameraSensorInfo.distortion().k3();
  double distortion_t1 = m_pbBufCameraSensorInfo.distortion().p1();
  double distortion_t2 = m_pbBufCameraSensorInfo.distortion().p2();

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg.d[0] = distortion_k1;
  camera_info_msg.d[1] = distortion_k2;
  camera_info_msg.d[2] = distortion_t1;
  camera_info_msg.d[3] = distortion_t2;
  camera_info_msg.d[4] = distortion_k3;

  // Original camera matrix
  camera_info_msg.k[0] = computed_focal_length;
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = cx_;
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = computed_focal_length;
  camera_info_msg.k[5] = cy_;
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;

  // rectification
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;

  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.p[0] = computed_focal_length;
  camera_info_msg.p[1] = 0.0;
  camera_info_msg.p[2] = cx_;
  camera_info_msg.p[3] = -computed_focal_length * hack_baseline;
  camera_info_msg.p[4] = 0.0;
  camera_info_msg.p[5] = computed_focal_length;
  camera_info_msg.p[6] = cy_;
  camera_info_msg.p[7] = 0.0;
  camera_info_msg.p[8] = 0.0;
  camera_info_msg.p[9] = 0.0;
  camera_info_msg.p[10] = 1.0;
  camera_info_msg.p[11] = 0.0;

  cameraInfoManager->setCameraInfo(camera_info_msg);
}

void CameraDriverSim::UpdateData(const int bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  auto simBridge = GetSimBridge(bridge_index);
  const bool succeeded = simBridge->Receive(&pBuffer, bufferLength, false);
  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

    // try reconnect1ion
    if (IsRunThread())
    {
      simBridge->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
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