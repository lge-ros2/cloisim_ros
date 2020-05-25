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
// #include <unistd.h>
// #include <math.h>
// #include <string>
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
  std::string part_name_;
  string camera_name_;
  vector<double> transform_;
  vector<string> camera_list_;

  get_parameter_or("sim.parts", part_name_, string("multi_camera"));

  get_parameter_or("camera_name", camera_name_, string("multi_camera"));
  get_parameter_or("transform", transform_, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  DBG_SIM_INFO("[CONFIG] sim.part:%s", part_name_.c_str());

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
    auto topic_name_ = camera_name_ + "/" + item_name + "/image_raw";
    DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_name_.c_str());
    pubImages.push_back(image_transport::create_publisher(GetNode(), topic_name_));
  }

  m_hashKeySub = GetRobotName() + part_name_;
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

void MultiCameraDriverSim::UpdateData()
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
    }
  }
}