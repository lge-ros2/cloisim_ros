/**
 *  @file   DepthCameraDriverSim.hpp
 *  @date   2020-07-09
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Depth                                                                    Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _DepthCameraDriverSim_H_
#define _DepthCameraDriverSim_H_

#include "driver_sim/driver_sim.hpp"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <protobuf/image_stamped.pb.h>

class DepthCameraDriverSim : public DriverSim
{
public:
  DepthCameraDriverSim();
  virtual ~DepthCameraDriverSim();

private:
  virtual void Initialize();
  virtual void Deinitialize();  
  virtual void UpdateData();

  void InitializeCameraInfoManager();

private:
  // key for connection
  std::string m_hashKeySub;

  // buffer from simulation
  gazebo::msgs::ImageStamped m_pbBuf;

  // message for ROS2 communictaion
  sensor_msgs::msg::Image msg_img;

  // Store current point cloud.
  // sensor_msgs::msg::PointCloud2 msg_pc2;

  // Camera info publisher
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubDepthCameraInfo{nullptr};

  // Depth Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;

  // Depth image publisher.
  image_transport::Publisher pubDepthImage;

  // Point cloud publisher.
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPointCloud;
};

#endif