/**
 *  @file   CameraDriverSim.hpp
 *  @date   2020-05-13
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _CameraDriverSim_H_
#define _CameraDriverSim_H_

#include "driver_sim/driver_sim.hpp"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <protobuf/image_stamped.pb.h>

class CameraDriverSim : public DriverSim
{
public:
  CameraDriverSim();
  virtual ~CameraDriverSim();

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

  // Camera info publisher
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubCameraInfo{nullptr};

  // Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;

  // Image publisher
  image_transport::Publisher pubImage;
};

#endif