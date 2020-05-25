/**
 *  @file   CameraDriverSim.cpp
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
#include "camera_driver_sim/CameraDriverSim.hpp"
#include "driver_sim/helper.h"
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace chrono_literals;

CameraDriverSim::CameraDriverSim()
    : DriverSim("camera_driver_sim")
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
  string part_name_;
  string frame_id_;
  string camera_name_;
  vector<double> transform_;

  get_parameter_or("sim.parts", part_name_, string("camera"));
  get_parameter_or("frame_id", frame_id_, string("camera_link"));
  get_parameter_or("camera_name", camera_name_, string("camera"));
  get_parameter_or("transform", transform_, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  const auto topic_name_ = camera_name_ + "/rgb/image_raw";
  m_hashKeySub = GetRobotName() + part_name_;

  DBG_SIM_INFO("[CONFIG] sim.part:%s", part_name_.c_str());
  DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_name_.c_str());
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

  pubImage = image_transport::create_publisher(GetNode(), topic_name_);

  GetSimBridge()->Connect(SimBridge::Mode::SUB, m_hashKeySub);
}

void CameraDriverSim::Deinitialize()
{
  pubImage.shutdown();

  GetSimBridge()->Disconnect();
}

void CameraDriverSim::UpdateData()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (IsRunThread())
  {
    const bool succeeded = GetSimBridge()->Receive(&pBuffer, bufferLength, false);
    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s", bufferLength, zmq_strerror(zmq_errno()));

      // try reconnect1ion
      if (IsRunThread())
      {
        GetSimBridge()->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
      }

      continue;
    }

    if (!m_pbBuf.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    m_simTime = rclcpp::Time(m_pbBuf.time().sec(), m_pbBuf.time().nsec());

    msg_img.header.stamp = m_simTime;

    const auto encoding_arg = GetImageEncondingType(m_pbBuf.image().pixel_format());
    const uint32_t cols_arg = m_pbBuf.image().width();
    const uint32_t rows_arg = m_pbBuf.image().height();
    const uint32_t step_arg = m_pbBuf.image().step();

    // Copy from src to image_msg
    sensor_msgs::fillImage(msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                           reinterpret_cast<const void *>(m_pbBuf.image().data().data()));

    pubImage.publish(msg_img);
  }
}