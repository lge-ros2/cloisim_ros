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
#include "driver_sim/helper.h"

using namespace std;
using namespace chrono_literals;

MultiCameraDriverSim::MultiCameraDriverSim()
    : DriverSim("multi_camera_driver_sim")
{
  Start();
}

MultiCameraDriverSim::~MultiCameraDriverSim()
{
  Stop();
}

void MultiCameraDriverSim::Initialize()
{
  string camera_name_;
  vector<double> transform_;
  vector<string> camera_list_;

  get_parameter_or("camera_name", camera_name_, string("multi_camera"));
  get_parameter_or("transform", transform_, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  get_parameter("camera_list", camera_list_);

  tf2::Quaternion fixed_rot;
  for (auto item : camera_list_)
  {
    string item_name;
    vector<double> transform_offset;
    string frame_id;

    get_parameter_or(item + ".name", item_name, string("noname"));
    get_parameter_or(item + ".transform_offset", transform_offset, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    get_parameter_or(item + ".frame_id", frame_id, string("noname_link"));

    transform_offset[0] += transform_[0];
    transform_offset[1] += transform_[1];
    transform_offset[2] += transform_[2];
    transform_offset[3] += transform_[3];
    transform_offset[4] += transform_[4];
    transform_offset[5] += transform_[5];

    geometry_msgs::msg::TransformStamped camera_tf;
    camera_tf.header.frame_id = "base_footprint";
    camera_tf.child_frame_id = frame_id;
    camera_tf.transform.translation.x = transform_offset[0];
    camera_tf.transform.translation.y = transform_offset[1];
    camera_tf.transform.translation.z = transform_offset[2];

    fixed_rot.setRPY(transform_offset[3], transform_offset[4], transform_offset[5]);

    camera_tf.transform.rotation.x = fixed_rot.x();
    camera_tf.transform.rotation.y = fixed_rot.y();
    camera_tf.transform.rotation.z = fixed_rot.z();
    camera_tf.transform.rotation.w = fixed_rot.w();

    AddStaticTf2(camera_tf);

    // Image publisher
    auto topic_base_name_ = camera_name_ + "/" + item_name;
    DBG_SIM_INFO("[CONFIG] topic_base_name:%s", topic_base_name_.c_str());

    pubImages.push_back(image_transport::create_publisher(GetNode(), topic_base_name_ + "/image_raw"));

    // Camera info publisher
    pubCamerasInfo.push_back(
      create_publisher<sensor_msgs::msg::CameraInfo>(topic_base_name_ + "/camera_info", rclcpp::SensorDataQoS()));

    InitializeCameraInfoManager(item);
  }

  m_hashKeySub = GetRobotName() + GetPartsName();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  GetSimBridge()->Connect(SimBridge::Mode::SUB, m_hashKeySub);
}

void MultiCameraDriverSim::Deinitialize()
{
  for (auto pub : pubImages)
  {
    pub.shutdown();
  }

  GetSimBridge()->Disconnect();
}

void MultiCameraDriverSim::InitializeCameraInfoManager(const string target_camera_item)
{
  string frame_id_, camera_name_;
  get_parameter_or(target_camera_item + ".name", camera_name_, string("camera"));
  get_parameter_or(target_camera_item + ".frame_id", frame_id_, string("camera_link"));

  int width_, height_;
  get_parameter_or(target_camera_item + ".camera_info.width", width_, int(0));
  get_parameter_or(target_camera_item + ".camera_info.height", height_, int(0));

  // C parameters
  auto cx_ = (static_cast<double>(width_) + 1.0) / 2.0;
  auto cy_ = (static_cast<double>(height_) + 1.0) / 2.0;

  double hfov_;
  get_parameter_or(target_camera_item + ".camera_info.hfov", hfov_, double(0.0));

  auto computed_focal_length = (static_cast<double>(width_)) / (2.0 * tan(hfov_ / 2.0));

  // CameraInfo
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.frame_id = frame_id_;
  camera_info_msg.height = height_;
  camera_info_msg.width = width_;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  const auto hack_baseline = 0.0f;

  // Get distortion from camera
  double distortion_k1{0.0};
  double distortion_k2{0.0};
  double distortion_k3{0.0};
  double distortion_t1{0.0};
  double distortion_t2{0.0};
  get_parameter_or(target_camera_item + ".camera_info.distortion.k1", distortion_k1, double(0.0));
  get_parameter_or(target_camera_item + ".camera_info.distortion.k2", distortion_k2, double(0.0));
  get_parameter_or(target_camera_item + ".camera_info.distortion.k3", distortion_k3, double(0.0));
  get_parameter_or(target_camera_item + ".camera_info.distortion.t1", distortion_t1, double(0.0));
  get_parameter_or(target_camera_item + ".camera_info.distortion.t2", distortion_t2, double(0.0));

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

  // Initialize camera_info_manager
  cameraInfoManager.push_back(std::make_shared<camera_info_manager::CameraInfoManager>(GetNode(), camera_name_));
  cameraInfoManager.back()->setCameraInfo(camera_info_msg);
}

void MultiCameraDriverSim::UpdateData()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetSimBridge()->Receive(&pBuffer, bufferLength, false);
  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

    // try reconnect1ion
    if (IsRunThread())
    {
      GetSimBridge()->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
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