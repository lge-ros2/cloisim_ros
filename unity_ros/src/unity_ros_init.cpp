/**
 *  @file   unity_ros_init.cpp
 *  @date   2020-04-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 node for controlling unity simulation
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "unity_ros/unity_ros_init.hpp"
#include "protobuf/param.pb.h"
#include "protobuf/any.pb.h"
#include "protobuf/time.pb.h"
#include <unistd.h>

using namespace std::literals;
using namespace UnityRos;
using namespace gazebo;

UnityRosInit::UnityRosInit()
  : Node("lidar_driver_sim",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  , m_pSimBridge(nullptr)
  , m_hashKey("")
  , throttler_(nullptr)
  , m_bRun(false)
{
  // Set Parameters from yaml file
  double publish_rate(10.0);
  std::string sim_ip("");
  int sim_manager_port(0);
  std::string model_name;
  get_parameter_or("sim.model", model_name, std::string("UnityRosInit"));
  get_parameter("sim.ip_address", sim_ip);
  get_parameter("sim.manager_port", sim_manager_port);
  get_parameter_or("publish_rate", publish_rate, 10.0);

  DBG_SIM_INFO("[CONFIG] sim manage ip:%s, port:%d, publish_rate:%f", sim_ip.c_str(), sim_manager_port, publish_rate);

  // set param for use_sim_time if not set by user already
  // if(!(hasParam("/use_sim_time")))
  //   setParam("/use_sim_time", true);

  throttler_ = new Throttler(publish_rate);

  m_pSimBridge = new SimBridge();

  if (m_pSimBridge)
  {
    m_pSimBridge->SetSimMasterAddress(sim_ip);
    m_pSimBridge->SetPortManagerPort(sim_manager_port);
  }

  // Offer transient local durability on the clock topic so that if publishing is infrequent (e.g.
  // the simulation is paused), late subscribers can receive the previously published message(s).
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

  Start();
}

UnityRosInit::~UnityRosInit()
{
  Stop();
  delete throttler_;
  delete m_pSimBridge;
}

void UnityRosInit::Start()
{
  m_hashKey = "UnityRosInit";
  DBG_SIM_MSG("TAG=[%s]", m_hashKey.c_str());

  if (m_pSimBridge)
  {
    m_pSimBridge->Connect(SimBridge::Mode::SUB, m_hashKey);
    m_bRun = true;
    m_thread = std::thread([=]() { RxProc(); });
  }
}

void UnityRosInit::Stop()
{
  m_bRun = false;
  usleep(100);

  if (m_thread.joinable())
  {
    m_thread.join();
  }

  m_pSimBridge->Disconnect();
}

void UnityRosInit::RxProc()
{
  static std::mutex mtxRosPub;
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (m_bRun)
  {
    const bool succeeded = m_pSimBridge->Receive(&pBuffer, bufferLength, false);

    if (succeeded == false || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error, return size(%d) %s!", bufferLength, zmq_strerror(zmq_errno()));

      // try reconnection
      m_pSimBridge->Disconnect();
      m_pSimBridge->Connect(SimBridge::Mode::SUB, m_hashKey);
      continue;
    }

    msgs::Param pbBuf;
    if (!pbBuf.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    if (pbBuf.name() == "timeInfo" &&
        pbBuf.value().type() == msgs::Any::NONE &&
        pbBuf.children_size() == 2)
    {
      const auto simTime =
        (pbBuf.children(0).name() != "simTime")?
          msgs::Time() : pbBuf.children(0).value().time_value();

      const auto realTime =
        (pbBuf.children(1).name() != "realTime")?
          msgs::Time() : pbBuf.children(1).value().time_value();

      PublishSimTime(rclcpp::Time(simTime.sec(), simTime.nsec()),
                     rclcpp::Time(realTime.sec(), realTime.nsec()));
    }
  }

  DBG_SIM_INFO("thread finished");
}

void UnityRosInit::PublishSimTime(const rclcpp::Time simTime, const rclcpp::Time realTime)
{
  if (throttler_->IsReady(realTime))
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = simTime;
    clock_pub_->publish(clock);
  }
}