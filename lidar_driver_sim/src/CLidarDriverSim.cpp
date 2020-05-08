/**
 *  @file   CLidarDriverSim.cpp
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

#include "lidar_driver_sim/CLidarDriverSim.hpp"
#include <unistd.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#define MAX_UPPER_ANGLE 3.1415926535
#define MAX_LOWER_ANGLE -MAX_UPPER_ANGLE

using namespace std::chrono_literals;

CLidarDriverSim::CLidarDriverSim()
  : Node("lidar_driver_sim",
        rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
  , m_bRun(false)
  , m_bIntensity(false)
  , m_fLowerAngle(MAX_LOWER_ANGLE)
  , m_fUpperAngle(MAX_UPPER_ANGLE)
{
  node_handle = std::shared_ptr<::rclcpp::Node>(this);

  this->static_tf_broadcaster
    = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle);

  std::string sim_ip("");
  int sim_manager_port(0);
  std::string robot_name_;
  std::string part_name_;
  std::string topic_name_;

  get_parameter("sim.ip_address", sim_ip);
  get_parameter("sim.manager_port", sim_manager_port);
  get_parameter_or("sim.model", robot_name_, std::string("CLOI_porter"));
  get_parameter_or("sim.parts", part_name_, std::string("front_lidar"));

  get_parameter_or("topic_name", topic_name_, std::string("scan"));
  get_parameter_or("frame_id", frame_id_, std::string("base_scan"));
  get_parameter_or("transform", transform_, std::vector<double>({0.27000, 0.0, 0.1124, 0.0, 0.0, 0.0}));
  get_parameter("intensity", m_bIntensity);

  get_parameter("filter.lower_angle", m_fLowerAngle);
  get_parameter("filter.upper_angle", m_fUpperAngle);

  DBG_SIM_INFO("[CONFIG] sim manage ip:%s, port:%d", sim_ip.c_str(), sim_manager_port);
  DBG_SIM_INFO("[CONFIG] intensity:%d, filter.lower_angle:%f, filter.upper_angle:%f",
      m_bIntensity, m_fLowerAngle, m_fUpperAngle);
  DBG_SIM_INFO("[CONFIG] sim.model:%s", robot_name_.c_str());
  DBG_SIM_INFO("[CONFIG] sim.part:%s", part_name_.c_str());
  DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_name_.c_str());
  DBG_SIM_INFO("[CONFIG] frame_id:%s", frame_id_.c_str());

  m_hashKeySub = robot_name_ + part_name_;
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  m_pSimBridge = new SimBridge();

  if (m_pSimBridge)
  {
    m_pSimBridge->SetSimMasterAddress(sim_ip);
    m_pSimBridge->SetPortManagerPort(sim_manager_port);
  }

  // ROS2 Publisher
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  pubLaser = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, qos);

  Start();
}

CLidarDriverSim::~CLidarDriverSim()
{
  m_bRun = false;
  usleep(100);

  if (m_thread.joinable())
  {
    m_thread.join();
  }

  m_pSimBridge->Disconnect();
}

void CLidarDriverSim::Start()
{
  m_pSimBridge->Connect(SimBridge::Mode::SUB, m_hashKeySub);
  m_bRun = true;
  m_thread = std::thread([=]() { ReadProc(); });

  auto callback_pub
    = [this]() -> void {
      UpdateStaticTF(m_simTime);
    };

  // ROS2 timer for Publisher
  timer = this->create_wall_timer(0.5s, callback_pub);
}

void CLidarDriverSim::ReadProc()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (m_bRun)
  {
    const bool succeeded = m_pSimBridge->Receive(&pBuffer, bufferLength, false);

    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s",
              bufferLength, zmq_strerror(zmq_errno()));

      // try reconnect1ion
      if(m_bRun) {
        m_pSimBridge->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
      }

      continue;
    }

    if (!m_pbBuf.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    m_simTime = rclcpp::Time(m_pbBuf.time().sec(), m_pbBuf.time().nsec());

    UpdateLaser(m_simTime);

    pubLaser->publish(msg_Laser);
  }
}

void CLidarDriverSim::UpdateStaticTF(const rclcpp::Time timestamp)
{
  geometry_msgs::msg::TransformStamped scan_tf;
  scan_tf.header.stamp = timestamp;
  scan_tf.header.frame_id = "base_link";
  scan_tf.child_frame_id = frame_id_;
  scan_tf.transform.translation.x = transform_[0];
  scan_tf.transform.translation.y = transform_[1];
  scan_tf.transform.translation.z = transform_[2];

  tf2::Quaternion convertQuternion;
  convertQuternion.setRPY( transform_[3], transform_[4], transform_[5] );
  convertQuternion = convertQuternion.normalize();

  scan_tf.transform.rotation.x = convertQuternion.x();
  scan_tf.transform.rotation.y = convertQuternion.y();
  scan_tf.transform.rotation.z = convertQuternion.z();
  scan_tf.transform.rotation.w = convertQuternion.w();

  this->static_tf_broadcaster->sendTransform(scan_tf);
}

void CLidarDriverSim::UpdateLaser(const rclcpp::Time timestamp)
{
  msg_Laser.header.stamp = timestamp;
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
    filter_beam_index_lower = (int)( (double)num_beams *
    ((m_fLowerAngle - msg_Laser.angle_min) /
     (msg_Laser.angle_max - msg_Laser.angle_min)) );
  }

  if (msg_Laser.angle_max > m_fUpperAngle)
  {
    filter_beam_index_upper = (int)( (double)num_beams *
    ((m_fUpperAngle - msg_Laser.angle_min) /
     (msg_Laser.angle_max - msg_Laser.angle_min)) );
  }

  if (m_bIntensity)
    msg_Laser.intensities.resize(num_beams);

  bool filter_out;
  for (int beam_idx = 0; beam_idx < num_beams; beam_idx++)
  {
    //printf("beam_idx:%d %f\n", beam_idx, m_pbBuf.scan().ranges(beam_idx));

    if ((filter_beam_index_lower > 0 && filter_beam_index_lower > beam_idx) ||
        (filter_beam_index_upper > 0 && filter_beam_index_upper < beam_idx) )
    {
      filter_out = true;
      //printf("beam_idx:%d filterd out\n", beam_idx);
    }
    else
      filter_out = false;


    msg_Laser.ranges[beam_idx]
      = (filter_out)? 0.0:m_pbBuf.scan().ranges(beam_idx);

    if (m_bIntensity)
    {
      msg_Laser.intensities[beam_idx]
        = (filter_out)? 0.0:m_pbBuf.scan().intensities(beam_idx);
    }
  }
}
