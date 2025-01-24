/**
 *  @file   contact.cpp
 *  @date   2025-01-23
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 contact class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2025 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_contact/contact.hpp"
#include <cloisim_ros_base/helper.hpp>
#include <geometry_msgs/msg/wrench.hpp>


using string = std::string;

namespace cloisim_ros
{
struct Quaternion
{
  double w, x, y, z;

  Quaternion conjugate() const {return {w, -x, -y, -z};}

  void normalize()
  {
    const auto norm = std::sqrt(w * w + x * x + y * y + z * z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
  }

  Quaternion operator*(const Quaternion & q) const
  {
    return {
      w * q.w - x * q.x - y * q.y - z * q.z,  // w
        w * q.x + x * q.w + y * q.z - z * q.y,  // x
        w * q.y + y * q.w + z * q.x - x * q.z,  // y
        w * q.z + z * q.w + x * q.y - y * q.x   // z
    };
  }

  cloisim::msgs::Vector3d rotateVectorReverse(const cloisim::msgs::Vector3d & vec)
  {
    // const Quaternion vec2quat = {0, vec.x(), vec.y(), vec.z()};
    // const auto q_conj = this->conjugate();
    // const auto q_result = q_conj * vec2quat * *this;

    cloisim::msgs::Vector3d result;
    result.set_x(vec.x());
    result.set_y(vec.y());
    result.set_z(vec.z());
    return result;
  }
};

Contact::Contact(
  const rclcpp::NodeOptions & options_, const string node_name,
  const string namespace_)
: Base(node_name, namespace_, options_)
{
  topic_name_ = "contacts";

  Start();
}

Contact::Contact(const string namespace_)
: Contact(rclcpp::NodeOptions(), "cloisim_ros_contact", namespace_)
{
}

Contact::~Contact() {Stop();}

void Contact::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  DBG_SIM_INFO("hashKey: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    // Get frame for message
    const auto frame_id = GetFrameId("contact_link");
    msg_contacts_state_.header.frame_id = frame_id;

    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);
  }

  // ROS2 Publisher
  const auto new_topic = GetPartsName() + "/" + topic_name_;
  pub_ =
    this->create_publisher<gazebo_msgs::msg::ContactsState>(new_topic, rclcpp::SensorDataQoS());

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&Contact::PublishData, this, std::placeholders::_1));
  }
}

void Contact::Convert()
{
  SetTime(pb_buf_.time());

  // Fill message with latest sensor data
  msg_contacts_state_.header.stamp = GetTime();
  msg_contacts_state_.states.clear();

  Quaternion identityQuat;
  const auto contactSize = pb_buf_.contact_size();
  for (auto i = 0; i < contactSize; i++) {
    const auto contact = pb_buf_.contact(i);

    // For each collision contact
    gazebo_msgs::msg::ContactState state;
    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();

    std::ostringstream stream;
    stream << "Debug:  i:(" << i << "/" << contactSize << ")"
           << "  my geom:" << state.collision1_name
           << "  other geom:" << state.collision2_name
           << "  time:" << msg_contacts_state_.header.stamp.sec
           << "." << msg_contacts_state_.header.stamp.nanosec;
    state.info = stream.str();

    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();

    // sum up all wrenches for each DOF
    geometry_msgs::msg::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    const auto contactGroupSize = contact.position_size();
    for (auto j = 0; j < contactGroupSize; ++j) {
      // Get force, torque and rotate into user specified frame.
      // identity is default for now if world is used.

      const auto forceVec = contact.wrench(j).body_1_wrench().force();
      const auto force = identityQuat.rotateVectorReverse(forceVec);

      // TODO(hyunseok-yang): not support in CLOiSim
      // const auto torqueVec = contact.wrench(j).body_1_wrench().torque();
      // const auto torque = identityQuat.rotateVectorReverse(torqueVec);

      // set wrenches
      geometry_msgs::msg::Wrench wrench;
      wrench.force.x = force.x();
      wrench.force.y = force.y();
      wrench.force.z = force.z();
      // wrench.torque.x = torque.x();
      // wrench.torque.y = torque.y();
      // wrench.torque.z = torque.z();
      state.wrenches.push_back(wrench);

      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      // total_wrench.torque.x += wrench.torque.x;
      // total_wrench.torque.y += wrench.torque.y;
      // total_wrench.torque.z += wrench.torque.z;

      // transform contact positions into relative frame
      const auto positionVec = contact.position(j);
      const auto position = identityQuat.rotateVectorReverse(positionVec);  // - frame_pos;

      geometry_msgs::msg::Vector3 contact_position;
      contact_position.x = position.x();
      contact_position.y = position.y();
      contact_position.z = position.z();
      state.contact_positions.push_back(contact_position);

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      const auto normalVec = contact.normal(j);
      const auto normal = identityQuat.rotateVectorReverse(normalVec);

      // set contact normals
      geometry_msgs::msg::Vector3 contact_normal;
      contact_normal.x = normal.x();
      contact_normal.y = normal.y();
      contact_normal.z = normal.z();
      state.contact_normals.push_back(contact_normal);

      state.depths.push_back(contact.depth(j));
    }

    state.total_wrench = total_wrench;

    msg_contacts_state_.states.push_back(state);
  }
}

void Contact::PublishData(const string & buffer)
{
  if (!pb_buf_.ParseFromString(buffer)) {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  Convert();

  pub_->publish(msg_contacts_state_);
}

}  // namespace cloisim_ros
