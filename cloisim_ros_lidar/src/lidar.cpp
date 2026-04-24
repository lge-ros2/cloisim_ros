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
#include <cloisim_msgs/param.pb.h>

#include <algorithm>

#include "cloisim_ros_lidar/lidar.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cloisim_ros_base/param_helper.hpp>

using namespace std::placeholders;
using string = std::string;

namespace cloisim_ros
{

Lidar::Lidar(const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: Base(node_name, namespace_, options_), pub_laser_(nullptr), pub_pc2_(nullptr)
{
  topic_name_ = "scan";

  Start();
}

Lidar::Lidar(const string namespace_)
: Lidar(rclcpp::NodeOptions(), "cloisim_ros_lidar", namespace_)
{
}

Lidar::~Lidar() {Stop();}

void Lidar::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  LOG_I(this, "hashKey: data(" << hashKeyData << ") info(" << hashKeyInfo << ")");

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  auto output_type = string("LaserScan");
  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    const auto frame_id = GetFrameId("base_scan");

    msg_laser_.header.frame_id = frame_id;
    msg_pc2_.header.frame_id = frame_id;

    auto parent_frame_id = std::string("base_link");
    auto transform_pose = GetObjectTransform(info_bridge_ptr, parent_frame_id);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose, parent_frame_id);

    output_type = GetOutputType(info_bridge_ptr);
  }

  // ROS2 Publisher
  if (output_type.compare("LaserScan") == 0) {
    pub_laser_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, rclcpp::SensorDataQoS());
  } else if (output_type.compare("PointCloud2Raw") == 0) {
    raw_point_cloud_ = true;
    pub_pc2_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, rclcpp::SensorDataQoS());
  } else if (output_type.compare("PointCloud2") == 0) {
    pub_pc2_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, rclcpp::SensorDataQoS());
  } else {
    DBG_SIM_ERR("Failed to create publisher, invalid output_type: %s", output_type.c_str());
  }

  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(data_bridge_ptr, bind(&Lidar::PublishData, this, _1, _2));
  }
}

string Lidar::GetOutputType(zmq::Bridge * const bridge_ptr)
{
  const auto reply = RequestReplyMessage(bridge_ptr, "request_output_type");

  if (
    reply.IsInitialized() && param::HasKey(reply, "output_type") &&
    param::GetValue(reply, "output_type").type() == cloisim::msgs::Any_ValueType_STRING &&
    !param::GetValue(reply, "output_type").string_value().empty())
  {
    const auto output_type = param::GetValue(reply, "output_type").string_value();
    DBG_SIM_INFO("output_type: %s", output_type.c_str());
    return output_type;
  }

  return "";
}

void Lidar::PublishData(const void * buffer, int bufferLength)
{
  if (!pb_buf_.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "##Parsing error, size=" << bufferLength);
    return;
  }

  SetTime(pb_buf_.header().stamp());

  if (pub_laser_ != nullptr) {
    UpdateLaserData();
    pub_laser_->publish(msg_laser_);
  } else if (pub_pc2_ != nullptr) {
    if (raw_point_cloud_) {
      UpdateRawPointCloudData();
    } else {
      UpdatePointCloudData();
    }
    pub_pc2_->publish(msg_pc2_);
  }
}

void Lidar::UpdatePointCloudData(const double min_intensity)
{
  // Pointcloud will be dense, unordered
  msg_pc2_.height = 1;
  msg_pc2_.is_dense = true;
  msg_pc2_.header.stamp = GetTime();

  // Cache values that are repeatedly used
  const auto beam_count = static_cast<uint32_t>(pb_buf_.count());
  const auto vertical_beam_count = static_cast<uint32_t>(pb_buf_.vertical_count());
  const auto angle_step = pb_buf_.angle_step();
  const auto vertical_angle_step = pb_buf_.vertical_angle_step();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  if (std::isinf(vertical_angle_step)) {
    DBG_SIM_WRN("Infinite angle step results in wrong PointCloud2");
  }

  // Initialize PC2 fields once (avoids re-creating every frame)
  if (!pc2_fields_initialized_) {
    sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);
    pcd_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    pc2_fields_initialized_ = true;
  }

  // Pre-compute trig tables when beam geometry changes
  if (beam_count != cached_beam_count_ || vertical_beam_count != cached_vertical_count_) {
    cached_beam_count_ = beam_count;
    cached_vertical_count_ = vertical_beam_count;

    cos_azimuth_.resize(beam_count);
    sin_azimuth_.resize(beam_count);
    auto azimuth = pb_buf_.angle_min();
    for (uint32_t i = 0; i < beam_count; ++i, azimuth += angle_step) {
      cos_azimuth_[i] = static_cast<float>(cos(azimuth));
      sin_azimuth_[i] = static_cast<float>(sin(azimuth));
    }

    cos_inclination_.resize(vertical_beam_count);
    sin_inclination_.resize(vertical_beam_count);
    double inclination = pb_buf_.vertical_angle_min();
    for (uint32_t j = 0; j < vertical_beam_count; ++j, inclination += vertical_angle_step) {
      cos_inclination_[j] = static_cast<float>(cos(inclination));
      sin_inclination_[j] = static_cast<float>(sin(inclination));
    }
  }

  // Resize once to max capacity
  const auto total_points = vertical_beam_count * beam_count;
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);
    pcd_modifier.resize(total_points);
  }

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pc2_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pc2_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pc2_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg_pc2_, "intensity");

  // Iterators to range and intensities
  auto range_iter = pb_buf_.ranges().begin();
  auto intensity_iter = pb_buf_.intensities().begin();

  // Number of points actually added
  size_t points_added = 0;

  // Fill pointcloud using precomputed trig tables
  for (uint32_t j = 0; j < vertical_beam_count; ++j) {
    const auto c_incl = cos_inclination_[j];
    const auto s_incl = sin_inclination_[j];

    for (uint32_t i = 0; i < beam_count;
      ++i, ++range_iter, ++intensity_iter)
    {
      auto r = *range_iter;
      // Skip NaN / inf points
      if (!std::isfinite(r)) {
        continue;
      }

      // Get intensity, clipping at min_intensity; default NaN to 0
      auto intensity = *intensity_iter;
      if (!std::isfinite(intensity)) {
        intensity = 0.0;
      } else if (intensity < min_intensity) {
        intensity = min_intensity;
      }

      // Convert spherical coordinates to Cartesian using precomputed trig
      *iter_x = r * c_incl * cos_azimuth_[i];
      *iter_y = r * c_incl * sin_azimuth_[i];
      *iter_z = r * s_incl;
      *iter_intensity = intensity;

      ++points_added;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  // Trim to actual size (single resize at end instead of initial + final)
  if (points_added != total_points) {
    sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);
    pcd_modifier.resize(points_added);
  }
}

void Lidar::UpdateRawPointCloudData(const double min_intensity)
{
  // PointCloud2Raw: ranges[] contains pre-computed xyz triples from the simulator.
  // Layout: [x0, y0, z0, x1, y1, z1, ...], size = count * 3
  // intensities[] contains one value per point, size = count.
  msg_pc2_.height = 1;
  msg_pc2_.is_dense = true;
  msg_pc2_.header.stamp = GetTime();

  const auto point_count = static_cast<uint32_t>(pb_buf_.count());
  // const auto ranges_size = pb_buf_.ranges_size();
  // const auto intensities_size = pb_buf_.intensities_size();

  // Initialize PC2 fields once
  if (!pc2_fields_initialized_) {
    sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);
    pcd_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    pc2_fields_initialized_ = true;
  }

  // Resize to max capacity
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);
    pcd_modifier.resize(point_count);
  }

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pc2_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pc2_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pc2_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg_pc2_, "intensity");

  auto range_iter = pb_buf_.ranges().begin();
  auto intensity_iter = pb_buf_.intensities().begin();

  size_t points_added = 0;

  for (uint32_t i = 0; i < point_count; ++i, ++intensity_iter) {
    const auto x = static_cast<float>(*(range_iter++));
    const auto y = static_cast<float>(*(range_iter++));
    const auto z = static_cast<float>(*(range_iter++));

    // Skip NaN points (rays that missed)
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    auto intensity = static_cast<float>(*intensity_iter);
    if (!std::isfinite(intensity)) {
      intensity = 0.0f;
    } else if (intensity < min_intensity) {
      intensity = static_cast<float>(min_intensity);
    }

    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    *iter_intensity = intensity;

    ++points_added;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }

  // Trim to actual size
  if (points_added != point_count) {
    sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pc2_);
    pcd_modifier.resize(points_added);
  }
}

void Lidar::UpdateLaserData(const double min_intensity)
{
  msg_laser_.header.stamp = GetTime();

  msg_laser_.angle_min = pb_buf_.angle_min();
  msg_laser_.angle_max = pb_buf_.angle_max();
  msg_laser_.angle_increment = pb_buf_.angle_step();
  msg_laser_.time_increment = 0;  // instantaneous simulator scan
  msg_laser_.scan_time = 0;       // not sure whether this is correct
  msg_laser_.range_min = pb_buf_.range_min();
  msg_laser_.range_max = pb_buf_.range_max();

  const auto beam_count = pb_buf_.count();
  const auto vertical_beam_count = pb_buf_.vertical_count();
  // DBG_SIM_INFO("num_beams:%d", num_beams);

  const auto start = (vertical_beam_count / 2) * beam_count;

  if (msg_laser_.ranges.size() != beam_count) {msg_laser_.ranges.resize(beam_count);}

  std::copy(
    pb_buf_.ranges().begin() + start, pb_buf_.ranges().begin() + start + beam_count,
    msg_laser_.ranges.begin());

  if (msg_laser_.intensities.size() != beam_count) {msg_laser_.intensities.resize(beam_count);}

  std::transform(
    pb_buf_.intensities().begin() + start,
    pb_buf_.intensities().begin() + start + beam_count, msg_laser_.intensities.begin(),
    [min_intensity](double i) -> double {return i > min_intensity ? i : min_intensity;});
}

}  // namespace cloisim_ros
