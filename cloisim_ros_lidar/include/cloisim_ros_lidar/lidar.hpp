/**
 *  @file   LidarDriverSim.hpp
 *  @date   2019-04-02
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 lidar class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_LIDAR__LIDAR_HPP_
#define CLOISIM_ROS_LIDAR__LIDAR_HPP_

#include <cloisim_msgs/laserscan_stamped.pb.h>

#include <string>
#include <vector>

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cloisim_ros
{
class Lidar : public Base
{
  using LaserScanPub = rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr;
  using PointCloud2Pub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

public:
  explicit Lidar(
    const rclcpp::NodeOptions & options_, const std::string node_name,
    const std::string namespace_ = "");
  explicit Lidar(const std::string namespace_ = "");
  ~Lidar();

private:
  void Initialize() override;
  void Deinitialize() override {}

private:
  std::string GetOutputType(zmq::Bridge * const bridge_ptr);

  void PublishData(const void* buffer, int bufferLength);
  void UpdatePointCloudData(const double min_intensity = 0.0);
  void UpdateRawPointCloudData(const double min_intensity = 0.0);
  void UpdateLaserData(const double min_intensity = 0.0);

private:
  // buffer from simulation
  cloisim::msgs::LaserScanStamped pb_buf_;

  // message for ROS2 communictaion
  sensor_msgs::msg::LaserScan msg_laser_;
  sensor_msgs::msg::PointCloud2 msg_pc2_;

  // Laser publisher
  LaserScanPub pub_laser_;
  PointCloud2Pub pub_pc2_;

  // Pre-computed trig tables for PointCloud2 conversion
  std::vector<float> cos_azimuth_;
  std::vector<float> sin_azimuth_;
  std::vector<float> cos_inclination_;
  std::vector<float> sin_inclination_;
  bool pc2_fields_initialized_ = false;
  uint32_t cached_beam_count_ = 0;
  uint32_t cached_vertical_count_ = 0;
  bool raw_point_cloud_ = false;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_LIDAR__LIDAR_HPP_
