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
#include <protobuf/image_stamped.pb.h>

class CameraDriverSim : public DriverSim
{
public:
  CameraDriverSim();
  virtual ~CameraDriverSim();

  virtual void Initialize();
  virtual void Deinitialize();

private:
  virtual void UpdateData();

private:
  // key for connection
  std::string m_hashKeySub;

  // buffer from simulation
  gazebo::msgs::ImageStamped m_pbBuf;

  // message for ROS2 communictaion
  sensor_msgs::msg::Image msg_img;

  // Image publisher
  image_transport::Publisher pubImage;
};

#endif