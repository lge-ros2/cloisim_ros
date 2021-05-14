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
    : Base(node_name_, namespace_, options_, 2)
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

  hashKeySub_ = GetMainHashKey();
  DBG_SIM_INFO("hash Key sub: %s", hashKeySub_.c_str());

  auto pBridgeData = GetBridge(0);
  auto pBridgeInfo = GetBridge(1);

  if (pBridgeData != nullptr)
  {
    pBridgeData->Connect(zmq::Bridge::Mode::SUB, portData, hashKeySub_ + "Data");
  }

  auto output_type = string("LaserScan");
  frame_id = GetFrameId("base_scan");
  if (pBridgeInfo != nullptr)
  {
    pBridgeInfo->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeySub_ + "Info");

    GetRos2Parameter(pBridgeInfo);

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
}

void Lidar::Deinitialize()
{
}

string Lidar::GetOutputType(zmq::Bridge* const pBridge)
{
  if (pBridge == nullptr)
  {
    return "";
  }

  msgs::Param request_msg;
  string serializedBuffer;

  request_msg.set_name("request_output_type");
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = pBridge->RequestReply(serializedBuffer);

  if (reply.size() <= 0)
  {
    DBG_SIM_ERR("Faild to get output type, length(%ld)", reply.size());
  }
  else
  {
    msgs::Param pbParam;
    if (pbParam.ParseFromString(reply) == false)
    {
      DBG_SIM_ERR("Faild to Parsing Proto buffer pBuffer(%p) length(%ld)", reply.data(), reply.size());
    }

    if (pbParam.IsInitialized() &&
        pbParam.name() == "output_type" &&
        pbParam.has_value() &&
        pbParam.value().type() == msgs::Any_ValueType_STRING &&
        !pbParam.value().string_value().empty())
    {
      const auto output_type = pbParam.value().string_value();
      DBG_SIM_INFO("output_type: %s", output_type.c_str());
      return output_type;
    }
  }

  return "";
}


void Lidar::UpdateData(const uint bridge_index)
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  const bool succeeded = GetBufferFromSimulator(bridge_index, &pBuffer, bufferLength);
  if (!succeeded || bufferLength < 0)
  {
    return;
  }

  if (!pbBuf_.ParseFromArray(pBuffer, bufferLength))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  m_simTime = rclcpp::Time(pbBuf_.time().sec(), pbBuf_.time().nsec());

  if (pubLaser != nullptr)
  {
    UpdateLaserData();
    pubLaser->publish(msg_laser);
  }
  else if (pubPC2 != nullptr)
  {
    UpdatePointCloudData();
    pubPC2->publish(msg_pc2);
  }
}

void Lidar::UpdatePointCloudData(const double min_intensity)
{
  // Pointcloud will be dense, unordered
  msg_pc2.height = 1;
  msg_pc2.is_dense = true;

  // Fill header
  msg_pc2.header.stamp = m_simTime;
  msg_pc2.header.frame_id = frame_id;

  // Cache values that are repeatedly used
  const auto beam_count = pbBuf_.scan().count();
  const auto vertical_beam_count = pbBuf_.scan().vertical_count();
  const auto angle_step = pbBuf_.scan().angle_step();
  const auto vertical_angle_step = pbBuf_.scan().vertical_angle_step();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  // Surprisingly, not setting the <vertical> tag results in nan instead of inf, which is ok
  if (std::isinf(vertical_angle_step))
  {
    DBG_SIM_WRN("Infinite angle step results in wrong PointCloud2");
  }

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2);

  pcd_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  pcd_modifier.resize(vertical_beam_count * beam_count);
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pc2, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg_pc2, "intensity");

  // Iterators to range and intensities
  auto range_iter = pbBuf_.scan().ranges().begin();
  auto intensity_iter = pbBuf_.scan().intensities().begin();

  // Number of points actually added
  size_t points_added = 0;

  // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
  double azimuth, inclination;

  // Index in vertical and horizontal loops
  size_t i, j;

  // Fill pointcloud with laser scan data, converting spherical to Cartesian
  for (j = 0, inclination = pbBuf_.scan().vertical_angle_min(); j < vertical_beam_count; ++j, inclination += vertical_angle_step)
  {
    auto c_inclination = cos(inclination);
    auto s_inclination = sin(inclination);

    for (i = 0, azimuth = pbBuf_.scan().angle_min(); i < beam_count; ++i, azimuth += angle_step, ++range_iter, ++intensity_iter)
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
  msg_laser.header.stamp = m_simTime;
  msg_laser.header.frame_id = frame_id;

  msg_laser.angle_min = pbBuf_.scan().angle_min();
  msg_laser.angle_max = pbBuf_.scan().angle_max();
  msg_laser.angle_increment = pbBuf_.scan().angle_step();
  msg_laser.time_increment = 0;
  msg_laser.scan_time = 0;
  msg_laser.range_min = pbBuf_.scan().range_min();
  msg_laser.range_max = pbBuf_.scan().range_max();

  const auto beam_count = pbBuf_.scan().count();
  const auto vertical_beam_count = pbBuf_.scan().vertical_count();
  //DBG_SIM_INFO("num_beams:%d", num_beams);

  const auto start = (vertical_beam_count / 2) * beam_count;

  if (msg_laser.ranges.size() != beam_count)
    msg_laser.ranges.resize(beam_count);

  std::copy(
      pbBuf_.scan().ranges().begin() + start,
      pbBuf_.scan().ranges().begin() + start + beam_count,
      msg_laser.ranges.begin());

  if (msg_laser.intensities.size() != beam_count)
    msg_laser.intensities.resize(beam_count);

  std::transform(
      pbBuf_.scan().intensities().begin() + start,
      pbBuf_.scan().intensities().begin() + start + beam_count,
      msg_laser.intensities.begin(),
      [min_intensity](double i) -> double { return i > min_intensity ? i : min_intensity; });
}