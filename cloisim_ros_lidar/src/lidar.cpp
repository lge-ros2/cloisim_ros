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
#include <cloisim_msgs/param.pb.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>

using namespace std;
using namespace cloisim;
using namespace cloisim_ros;

Lidar::Lidar(const rclcpp::NodeOptions &options_, const string node_name_, const string namespace_)
    : Base(node_name_, namespace_, options_)
    , pubLaser(nullptr)
    , pubPC2(nullptr)
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
  DBG_SIM_INFO("hash Key: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto pBridgeData = CreateBridge(hashKeyData);
  auto pBridgeInfo = CreateBridge(hashKeyInfo);

  auto output_type = string("LaserScan");
  if (pBridgeInfo != nullptr)
  {
    pBridgeInfo->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(pBridgeInfo);

    frame_id = GetFrameId("base_scan");

    const auto transform = GetObjectTransform(pBridgeInfo);
    SetupStaticTf2(transform, frame_id);

    output_type = GetOutputType(pBridgeInfo);
  }

  // ROS2 Publisher
  if (output_type.compare("LaserScan") == 0)
  {
    pubLaser = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rclcpp::SensorDataQoS());

  }
  else if (output_type.compare("PointCloud2") == 0)
  {
    pubPC2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, rclcpp::SensorDataQoS());
  }
  else
  {
    DBG_SIM_ERR("Failed to create publisher, invalid output_type: %s", output_type.c_str());
  }

  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    CreatePublisherThread(pBridgeData);
  }
}

string Lidar::GetOutputType(zmq::Bridge* const pBridge)
{
  const auto reply = RequestReplyMessage(pBridge, "request_output_type");

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


void Lidar::UpdatePublishingData(const string &buffer)
{
  if (!pbBuf.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetSimTime(pbBuf.time());

  if (pubLaser != nullptr)
  {
    UpdateLaserData();
    pubLaser->publish(msgLaser);
  }
  else if (pubPC2 != nullptr)
  {
    UpdatePointCloudData();
    pubPC2->publish(msgPC2);
  }
}

void Lidar::UpdatePointCloudData(const double min_intensity)
{
  // Pointcloud will be dense, unordered
  msgPC2.height = 1;
  msgPC2.is_dense = true;

  // Fill header
  msgPC2.header.stamp = GetSimTime();
  msgPC2.header.frame_id = frame_id;

  // Cache values that are repeatedly used
  const auto beam_count = pbBuf.scan().count();
  const auto vertical_beam_count = pbBuf.scan().vertical_count();
  const auto angle_step = pbBuf.scan().angle_step();
  const auto vertical_angle_step = pbBuf.scan().vertical_angle_step();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  // Surprisingly, not setting the <vertical> tag results in nan instead of inf, which is ok
  if (std::isinf(vertical_angle_step))
  {
    DBG_SIM_WRN("Infinite angle step results in wrong PointCloud2");
  }

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(msgPC2);

  pcd_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  pcd_modifier.resize(vertical_beam_count * beam_count);
  sensor_msgs::PointCloud2Iterator<float> iter_x(msgPC2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msgPC2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msgPC2, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(msgPC2, "intensity");

  // Iterators to range and intensities
  auto range_iter = pbBuf.scan().ranges().begin();
  auto intensity_iter = pbBuf.scan().intensities().begin();

  // Number of points actually added
  size_t points_added = 0;

  // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
  double azimuth, inclination;

  // Index in vertical and horizontal loops
  size_t i, j;

  // Fill pointcloud with laser scan data, converting spherical to Cartesian
  for (j = 0, inclination = pbBuf.scan().vertical_angle_min(); j < vertical_beam_count; ++j, inclination += vertical_angle_step)
  {
    auto c_inclination = cos(inclination);
    auto s_inclination = sin(inclination);

    for (i = 0, azimuth = pbBuf.scan().angle_min(); i < beam_count; ++i, azimuth += angle_step, ++range_iter, ++intensity_iter)
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
  msgLaser.header.stamp = GetSimTime();
  msgLaser.header.frame_id = frame_id;

  msgLaser.angle_min = pbBuf.scan().angle_min();
  msgLaser.angle_max = pbBuf.scan().angle_max();
  msgLaser.angle_increment = pbBuf.scan().angle_step();
  msgLaser.time_increment = 0;
  msgLaser.scan_time = 0;
  msgLaser.range_min = pbBuf.scan().range_min();
  msgLaser.range_max = pbBuf.scan().range_max();

  const auto beam_count = pbBuf.scan().count();
  const auto vertical_beam_count = pbBuf.scan().vertical_count();
  //DBG_SIM_INFO("num_beams:%d", num_beams);

  const auto start = (vertical_beam_count / 2) * beam_count;

  if (msgLaser.ranges.size() != beam_count)
    msgLaser.ranges.resize(beam_count);

  std::copy(
      pbBuf.scan().ranges().begin() + start,
      pbBuf.scan().ranges().begin() + start + beam_count,
      msgLaser.ranges.begin());

  if (msgLaser.intensities.size() != beam_count)
    msgLaser.intensities.resize(beam_count);

  std::transform(
      pbBuf.scan().intensities().begin() + start,
      pbBuf.scan().intensities().begin() + start + beam_count,
      msgLaser.intensities.begin(),
      [min_intensity](double i) -> double { return i > min_intensity ? i : min_intensity; });
}