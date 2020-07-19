/**
 *  @file   LidarDriverSim.cpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Lidar Driver class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "lidar_driver_sim/LidarDriverSim.hpp"
#include <unistd.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#define MAX_UPPER_ANGLE 3.1415926535
#define MAX_LOWER_ANGLE -MAX_UPPER_ANGLE

using namespace std;
using namespace chrono_literals;

LidarDriverSim::LidarDriverSim()
    : DriverSim("lidar_driver_sim"),
    m_bIntensity(false),
    m_fLowerAngle(MAX_LOWER_ANGLE),
    m_fUpperAngle(MAX_UPPER_ANGLE)
{
  Start();
}

LidarDriverSim::~LidarDriverSim()
{
  Stop();
}

void LidarDriverSim::Initialize()
{
  string topic_name_;
  vector<double> transform_;

  get_parameter_or("topic_name", topic_name_, string("scan"));
  get_parameter_or("frame_id", frame_id_, string("base_scan"));
  get_parameter_or("transform", transform_, vector<double>({0, 0, 0, 0, 0, 0}));

  get_parameter("intensity", m_bIntensity);
  get_parameter("filter.lower_angle", m_fLowerAngle);
  get_parameter("filter.upper_angle", m_fUpperAngle);

  DBG_SIM_INFO("[CONFIG] intensity: %d, filter.lower_angle: %f, filter.upper_angle: %f",
               m_bIntensity, m_fLowerAngle, m_fUpperAngle);
  DBG_SIM_INFO("[CONFIG] topic_name: %s", topic_name_.c_str());
  DBG_SIM_INFO("[CONFIG] frame_id: %s", frame_id_.c_str());

  m_hashKeySub = GetRobotName() + GetPartsName();
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  geometry_msgs::msg::TransformStamped scan_tf;
  tf2::Quaternion convertQuternion;
  convertQuternion.setRPY(transform_[3], transform_[4], transform_[5]);
  convertQuternion = convertQuternion.normalize();

  scan_tf.header.frame_id = "base_link";
  scan_tf.child_frame_id = frame_id_;
  scan_tf.transform.translation.x = transform_[0];
  scan_tf.transform.translation.y = transform_[1];
  scan_tf.transform.translation.z = transform_[2];
  scan_tf.transform.rotation.x = convertQuternion.x();
  scan_tf.transform.rotation.y = convertQuternion.y();
  scan_tf.transform.rotation.z = convertQuternion.z();
  scan_tf.transform.rotation.w = convertQuternion.w();

  AddStaticTf2(scan_tf);

  // ROS2 Publisher
  pubLaser = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rclcpp::SensorDataQoS());

  GetSimBridge()->Connect(SimBridge::Mode::SUB, m_hashKeySub);
}

void LidarDriverSim::Deinitialize()
{
  GetSimBridge()->Disconnect();
}

void LidarDriverSim::UpdateData()
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

    UpdateLaserData();

    pubLaser->publish(msg_Laser);
  }
}

void LidarDriverSim::UpdateLaserData()
{
  msg_Laser.header.stamp = m_simTime;
  msg_Laser.header.frame_id = frame_id_;

  const int num_beams = (int)m_pbBuf.scan().count();
  //DBG_SIM_INFO("num_beams:%d", num_beams);

  if (num_beams <= 0)
  {
    return;
  }

  msg_Laser.angle_min = m_pbBuf.scan().angle_min();
  msg_Laser.angle_max = m_pbBuf.scan().angle_max();
  msg_Laser.angle_increment = m_pbBuf.scan().angle_step();
  msg_Laser.scan_time = 0; //getScanPeriod();
  msg_Laser.time_increment = 0; //getTimeIncrement();
  msg_Laser.range_min = m_pbBuf.scan().range_min();
  msg_Laser.range_max = m_pbBuf.scan().range_max();

  msg_Laser.ranges.resize(num_beams);

  // calculate angle filter range
  int filter_beam_index_lower = -1;
  int filter_beam_index_upper = -1;

  if (msg_Laser.angle_min < m_fLowerAngle)
  {
    filter_beam_index_lower = (int)((double)num_beams *
                                    ((m_fLowerAngle - msg_Laser.angle_min) /
                                     (msg_Laser.angle_max - msg_Laser.angle_min)));
  }

  if (msg_Laser.angle_max > m_fUpperAngle)
  {
    filter_beam_index_upper = (int)((double)num_beams *
                                    ((m_fUpperAngle - msg_Laser.angle_min) /
                                     (msg_Laser.angle_max - msg_Laser.angle_min)));
  }

  if (m_bIntensity)
    msg_Laser.intensities.resize(num_beams);

  bool filter_out;
  for (int beam_idx = 0; beam_idx < num_beams; beam_idx++)
  {
    //printf("beam_idx:%d %f\n", beam_idx, m_pbBuf.scan().ranges(beam_idx));

    if ((filter_beam_index_lower > 0 && filter_beam_index_lower > beam_idx) ||
        (filter_beam_index_upper > 0 && filter_beam_index_upper < beam_idx))
    {
      filter_out = true;
      //printf("beam_idx:%d filterd out\n", beam_idx);
    }
    else
    {
      filter_out = false;
    }

    msg_Laser.ranges[beam_idx] = (filter_out)? 0.0 : m_pbBuf.scan().ranges(beam_idx);

    if (m_bIntensity)
    {
      msg_Laser.intensities[beam_idx] = (filter_out)? 0.0 : m_pbBuf.scan().intensities(beam_idx);
    }
  }
}