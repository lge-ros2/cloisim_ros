/**
 *  @file   Lidar.cpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Lidar class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_lidar/lidar.hpp"
#include <algorithm>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cloisim_msgs/param.pb.h>

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;

Lidar::Lidar(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
    , pub_laser_(nullptr)
    , pub_pc2_(nullptr)
{
  topic_name_ = "scan";

  Start();
}

Lidar::Lidar(const string namespace_)
    : Lidar(rclcpp::NodeOptions(), "cloisim_ros_lidar", namespace_)
{
}

Lidar::~Lidar()
{
  Stop();
}

void Lidar::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  DBG_SIM_INFO("hashKey: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  auto output_type = string("LaserScan");
  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    const auto frame_id = GetFrameId("base_scan");

    msg_laser_.header.frame_id = frame_id;
    msg_pc2_.header.frame_id = frame_id;

    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);

    output_type = GetOutputType(info_bridge_ptr);
  }

  // ROS2 Publisher
  if (output_type.compare("LaserScan") == 0)
  {
    pub_laser_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rclcpp::SensorDataQoS());
  }
  else if (output_type.compare("PointCloud2") == 0)
  {
    pub_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, rclcpp::SensorDataQoS());
  }
  else
  {
    DBG_SIM_ERR("Failed to create publisher, invalid output_type: %s", output_type.c_str());
  }

  if (data_bridge_ptr != nullptr)
  {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&Lidar::PublishData, this, std::placeholders::_1));
  }
}

string Lidar::GetOutputType(zmq::Bridge *const bridge_ptr)
{
  const auto reply = RequestReplyMessage(bridge_ptr, "request_output_type");

  if (reply.IsInitialized() &&
      (reply.name().compare("output_type") == 0) &&
      reply.has_value() && reply.value().type() == msgs::Any_ValueType_STRING &&
      !reply.value().string_value().empty())
  {
    const auto output_type = reply.value().string_value();
    DBG_SIM_INFO("output_type: %s", output_type.c_str());
    return output_type;
  }

  return "";
}

void Lidar::PublishData(const string &buffer)
{
  if (!pb_buf_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.time());

  if (pub_laser_ != nullptr)
  {
    UpdateLaserData();
    pub_laser_->publish(msg_laser_);
  }
  else if (pub_pc2_ != nullptr)
  {
    UpdatePointCloudData();
    pub_pc2_->publish(msg_pc2_);
  }
}

void Lidar::UpdatePointCloudData(const double min_intensity)
{
  // Pointcloud will be dense, unordered
  msg_pc2_.height = 1;
  msg_pc2_.is_dense = true;

  // Fill header
  msg_pc2_.header.stamp = GetTime();

  // Cache values that are repeatedly used
  const auto beam_count = pb_buf_.scan().count();
  const auto vertical_beam_count = pb_buf_.scan().vertical_count();
  const auto angle_step = pb_buf_.scan().angle_step();
  const auto vertical_angle_step = pb_buf_.scan().vertical_angle_step();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  // Surprisingly, not setting the <vertical> tag results in nan instead of inf, which is ok
  if (std::isinf(vertical_angle_step))
  {
    DBG_SIM_WRN("Infinite angle step results in wrong PointCloud2");
  }

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);

  pcd_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  pcd_modifier.resize(vertical_beam_count * beam_count);
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pc2_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pc2_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pc2_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg_pc2_, "intensity");

  // Iterators to range and intensities
  auto range_iter = pb_buf_.scan().ranges().begin();
  auto intensity_iter = pb_buf_.scan().intensities().begin();

  // Number of points actually added
  size_t points_added = 0;

  // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
  double azimuth, inclination;

  // Index in vertical and horizontal loops
  size_t i, j;

  // Fill pointcloud with laser scan data, converting spherical to Cartesian
  for (j = 0, inclination = pb_buf_.scan().vertical_angle_min(); j < vertical_beam_count; ++j, inclination += vertical_angle_step)
  {
    auto c_inclination = cos(inclination);
    auto s_inclination = sin(inclination);

    for (i = 0, azimuth = pb_buf_.scan().angle_min(); i < beam_count; ++i, azimuth += angle_step, ++range_iter, ++intensity_iter)
    {
      auto c_azimuth = cos(azimuth);
      auto s_azimuth = sin(azimuth);

      auto r = *range_iter;
      // Skip NaN / inf points
      if (!std::isfinite(r))
      {
        continue;
      }

      // Get intensity, clipping at min_intensity
      auto intensity = *intensity_iter;
      if (intensity < min_intensity)
      {
        intensity = min_intensity;
      }

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *iter_x = r * c_inclination * c_azimuth;
      *iter_y = r * c_inclination * s_azimuth;
      *iter_z = r * s_inclination;
      *iter_intensity = intensity;

      // Increment ouput iterators
      ++points_added;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  pcd_modifier.resize(points_added);
}

void Lidar::UpdateLaserData(const double min_intensity)
{
  msg_laser_.header.stamp = GetTime();

  msg_laser_.angle_min = pb_buf_.scan().angle_min();
  msg_laser_.angle_max = pb_buf_.scan().angle_max();
  msg_laser_.angle_increment = pb_buf_.scan().angle_step();
  msg_laser_.time_increment = 0;  // instantaneous simulator scan
  msg_laser_.scan_time = 0;       // not sure whether this is correct
  msg_laser_.range_min = pb_buf_.scan().range_min();
  msg_laser_.range_max = pb_buf_.scan().range_max();

  const auto beam_count = pb_buf_.scan().count();
  const auto vertical_beam_count = pb_buf_.scan().vertical_count();
  // DBG_SIM_INFO("num_beams:%d", num_beams);

  const auto start = (vertical_beam_count / 2) * beam_count;

  if (msg_laser_.ranges.size() != beam_count)
    msg_laser_.ranges.resize(beam_count);

  std::copy(
      pb_buf_.scan().ranges().begin() + start,
      pb_buf_.scan().ranges().begin() + start + beam_count,
      msg_laser_.ranges.begin());

  if (msg_laser_.intensities.size() != beam_count)
    msg_laser_.intensities.resize(beam_count);

  std::transform(
      pb_buf_.scan().intensities().begin() + start,
      pb_buf_.scan().intensities().begin() + start + beam_count,
      msg_laser_.intensities.begin(),
      [min_intensity](double i) -> double
      { return i > min_intensity ? i : min_intensity; });
}