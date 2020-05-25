/**
 *  @file   MultiCameraDriverSim.hpp
 *  @date   2020-05-20
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Multi Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _MultiCameraDriverSim_H_
#define _MultiCameraDriverSim_H_

#include "driver_sim/driver_sim.hpp"
#include <image_transport/image_transport.h>
#include <protobuf/images_stamped.pb.h>

class MultiCameraDriverSim : public DriverSim
{
public:
  MultiCameraDriverSim();
  virtual ~MultiCameraDriverSim();

  virtual void Initialize();

private:
  virtual void Deinitialize();
  virtual void UpdateData();

private:
  std::string m_hashKeySub;

  // Yaml parameters
  bool m_bIntensity;

  // buffer from simulation
  gazebo::msgs::ImagesStamped m_pbBuf;

  // message for ROS2 communictaion
  sensor_msgs::msg::Image msg_img;

  // Laser publisher
  std::vector<image_transport::Publisher> pubImages;
};
#endif