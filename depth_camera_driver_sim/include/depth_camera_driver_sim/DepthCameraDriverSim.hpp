/**
 *  @file   DepthCameraDriverSim.hpp
 *  @date   2020-07-09
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Depth Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _DepthCameraDriverSim_H_
#define _DepthCameraDriverSim_H_

#include <camera_driver_sim/CameraDriverSim.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class DepthCameraDriverSim : public CameraDriverSim
{
public:
  DepthCameraDriverSim();
  virtual ~DepthCameraDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index) override;

private:
  // Camera info publisher
  // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubDepthCameraInfo{nullptr};

  // Store current point cloud.
  // sensor_msgs::msg::PointCloud2 msg_pc2;

  // Point cloud publisher.
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPointCloud;
};

#endif