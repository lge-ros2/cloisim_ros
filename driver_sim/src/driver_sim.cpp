/**
 *  @file   driver_sim.cpp
 *  @date   2020-05-22
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 driver Sim base base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <driver_sim/driver_sim.hpp>

using namespace std;
using namespace gazebo;

DriverSim::DriverSim(const string name, const int number_of_simbridge)
    : Node(name,
           rclcpp::NodeOptions()
               .parameter_overrides(vector<rclcpp::Parameter>{rclcpp::Parameter("use_sim_time", true)})
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)),
      m_bRunThread(false), m_node_handle(shared_ptr<rclcpp::Node>(this, [](auto) {}))
{
  m_simBridgeList.reserve(number_of_simbridge);

  for (auto index = 0; index < number_of_simbridge; index++)
  {
    auto pSimBridge = new SimBridge();
    m_simBridgeList.push_back(pSimBridge);
  }

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node_handle);
  m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node_handle);
}

DriverSim::~DriverSim()
{
  // DBG_SIM_INFO("Delete");
  for (auto pSimBridge : m_simBridgeList)
  {
    delete pSimBridge;
  }
}

void DriverSim::Start(const bool runSingleDataThread)
{
  m_bRunThread = true;

  Initialize();

  if (runSingleDataThread)
  {
    m_thread = thread([=]() {
      while (IsRunThread()) {
        UpdateData();
      }});
  }

  auto callback_pub = [this]() -> void {
    PublishStaticTF();
  };

  // ROS2 timer for Publisher
  m_timer = this->create_wall_timer(0.5s, callback_pub);
}

void DriverSim::Stop()
{
  m_bRunThread = false;

  usleep(200);

  if (m_thread.joinable())
  {
    m_thread.join();
    // DBG_SIM_INFO("Thread finished");
  }

  Deinitialize();
}

void DriverSim::SetupStaticTf2Message(const gazebo::msgs::Pose transform, const string child_frame_id, const string parent_frame_id)
{
  geometry_msgs::msg::TransformStamped msg_tf;
  msg_tf.header.frame_id = parent_frame_id;
  msg_tf.child_frame_id = child_frame_id;
  msg_tf.transform.translation.x = transform.position().x();
  msg_tf.transform.translation.y = transform.position().y();
  msg_tf.transform.translation.z = transform.position().z();
  msg_tf.transform.rotation.x = transform.orientation().x();
  msg_tf.transform.rotation.y = transform.orientation().y();
  msg_tf.transform.rotation.z = transform.orientation().z();
  msg_tf.transform.rotation.w = transform.orientation().w();

  AddStaticTf2(msg_tf);
}

void DriverSim::PublishTF()
{
  m_tf_broadcaster->sendTransform(m_tf_list);
  m_tf_list.clear();
}

void DriverSim::PublishStaticTF()
{
  // Update timestamp
  for (auto _tf : m_static_tf_list)
  {
    _tf.header.stamp = m_simTime;
  }

  m_static_tf_broadcaster->sendTransform(m_static_tf_list);
}

SimBridge* DriverSim::GetSimBridge(const uint bridge_index)
{
  if (bridge_index >= m_simBridgeList.capacity())
  {
    DBG_SIM_WRN("Wrong bridge index(%d) / total sim bridges(%lu)", bridge_index, m_simBridgeList.capacity());
    return nullptr;
  }

  return m_simBridgeList.at(bridge_index);
}

void DriverSim::DisconnectSimBridges()
{
  for (auto pSimBridge : m_simBridgeList)
  {
    pSimBridge->Disconnect();
  }
}

msgs::Pose DriverSim::GetObjectTransform(const int bridge_index, const string target_name)
{
  auto const pSimBridge = GetSimBridge(bridge_index);
  return GetObjectTransform(pSimBridge, target_name);
}

msgs::Pose DriverSim::GetObjectTransform(SimBridge* const pSimBridge, const string target_name)
{
  msgs::Pose transform;
  transform.Clear();

  if (pSimBridge == nullptr)
  {
    return transform;
  }

  msgs::Param request_msg;
  request_msg.set_name("request_transform");

  auto pVal = request_msg.mutable_value();
  pVal->set_type(msgs::Any::STRING);
  pVal->set_string_value(target_name);

  string serializedBuffer;
  request_msg.SerializeToString(&serializedBuffer);

  pSimBridge->Send(serializedBuffer.data(), serializedBuffer.size());

  void *pBuffer = nullptr;
  int bufferLength = 0;
  const auto succeeded = pSimBridge->Receive(&pBuffer, bufferLength);

  if (!succeeded || bufferLength < 0)
  {
    DBG_SIM_ERR("Faild to get object transform, length(%d)", bufferLength);
  }
  else
  {
    msgs::Param m_pbBufParam;
    if (m_pbBufParam.ParseFromArray(pBuffer, bufferLength) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%d)", pBuffer, bufferLength);
    }

    if (m_pbBufParam.IsInitialized() &&
        m_pbBufParam.name() == "transform" &&
        m_pbBufParam.has_value())
    {
      transform.CopyFrom(m_pbBufParam.value().pose3d_value());
    }
  }

  return transform;
}

void DriverSim::GetRos2Parameter(SimBridge* const pSimBridge)
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
      auto param0 = m_pbBufParam.children(0);
      if (param0.name() == "topic_name" && param0.has_value())
      {
        topic_name_ = param0.value().string_value();
      }

      auto param1 = m_pbBufParam.children(1);
      if (param1.name() == "frame_id" && param1.has_value())
      {
        frame_id_ = param1.value().string_value();
      }
    }

    DBG_SIM_INFO("[CONFIG] topic_name: %s", topic_name_.c_str());
    DBG_SIM_INFO("[CONFIG] frame_id: %s", frame_id_.c_str());
  }
}