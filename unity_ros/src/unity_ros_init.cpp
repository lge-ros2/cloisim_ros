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
#include <protobuf/param.pb.h>
#include <protobuf/any.pb.h>
#include <protobuf/time.pb.h>
#include <unistd.h>

using namespace std;
using namespace literals;
using namespace UnityRos;
using namespace gazebo;

UnityRosInit::UnityRosInit()
  : Node("unity_ros_init",
      rclcpp::NodeOptions()
        .parameter_overrides(vector<rclcpp::Parameter>{rclcpp::Parameter("use_sim_time", true)})
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  , m_pSimBridge(new SimBridge())
  , throttler_(nullptr)
  , m_bRun(false)
{
  string model_name;
  get_parameter_or("model", model_name, string("world"));
  get_parameter_or("bridge.Clock", portClock_, uint16_t(0));

  m_hashKey = model_name + get_name();

  double publish_rate(10.0);
  get_parameter_or("publish_rate", publish_rate, 10.0);
  DBG_SIM_INFO("[CONFIG] publish_rate:%f", publish_rate);

  throttler_ = new Throttler(publish_rate);

  // Offer transient local durability on the clock topic so that if publishing is infrequent,
  // late subscribers can receive the previously published message(s).
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));

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
  DBG_SIM_MSG("TAG=[%s]", m_hashKey.c_str());

  if (m_pSimBridge)
  {
    m_pSimBridge->Connect(SimBridge::Mode::SUB, portClock_, m_hashKey + "Clock");
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
      m_pSimBridge->Reconnect(SimBridge::Mode::SUB, portClock_, m_hashKey);
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
      const auto simTime = (pbBuf.children(0).name() != "simTime") ? msgs::Time() : pbBuf.children(0).value().time_value();

      const auto realTime = (pbBuf.children(1).name() != "realTime") ? msgs::Time() : pbBuf.children(1).value().time_value();

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