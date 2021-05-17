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
    explicit Lidar(const rclcpp::NodeOptions &options_, const std::string node_name_, const std::string namespace_ = "");
    explicit Lidar(const std::string namespace_ = "");
    ~Lidar();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdatePublishingData(const std::string &buffer) override;

  private:
    std::string GetOutputType(zmq::Bridge* const pBridge);
    void UpdatePointCloudData(const double min_intensity = 0.0);
    void UpdateLaserData(const double min_intensity = 0.0);

  private:
    std::string frame_id;

    // buffer from simulation
    cloisim::msgs::LaserScanStamped pbBuf;

    // message for ROS2 communictaion
    sensor_msgs::msg::LaserScan msgLaser;
    sensor_msgs::msg::PointCloud2 msgPC2;

    // Laser publisher
    LaserScanPub pubLaser;
    PointCloud2Pub pubPC2;
  };
}
#endif