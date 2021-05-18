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

#ifndef _CLOISIM_ROS_LIDAR_HPP_
#define _CLOISIM_ROS_LIDAR_HPP_

#include <cloisim_ros_base/base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cloisim_msgs/laserscan_stamped.pb.h>

namespace cloisim_ros
{
  class Lidar : public Base
  {
    using LaserScanPub = rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr;
    using PointCloud2Pub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

  public:
    explicit Lidar(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
    explicit Lidar(const std::string namespace_ = "");
    ~Lidar();

  private:
    void Initialize() override;
    void Deinitialize() override { };
    void UpdatePublishingData(const std::string &buffer) override;

  private:
    std::string GetOutputType(zmq::Bridge* const bridge_ptr);
    void UpdatePointCloudData(const double min_intensity = 0.0);
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
  };
}
#endif