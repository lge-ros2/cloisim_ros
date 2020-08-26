/**
 *  @file   RealSenseDriverSim.hpp
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
#ifndef _RealSenseDriverSim_H_
#define _RealSenseDriverSim_H_

#include <driver_sim/driver_sim.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <protobuf/image_stamped.pb.h>
#include <protobuf/camerasensor.pb.h>

class RealSenseDriverSim : public DriverSim
{
public:
  RealSenseDriverSim();
  virtual ~RealSenseDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index) override;
  virtual void SetupStaticTf2Message(const gazebo::msgs::Pose transform, const std::string frame_id) override;

  void GetCameraSensorMessage(SimBridge* const pSimBridge);
  void InitializeCameraInfoMessage(const std::string frame_id);

private:
  static const int max_modules = 4;

  std::string mainframe_id;

  std::vector<std::thread> m_threads;

  // key for connection
  std::vector<std::string> m_hashKeySubs;

  // buffer from simulation
  std::vector<gazebo::msgs::ImageStamped> m_pbBuf;

  // message for ROS2 communictaion
  std::vector<sensor_msgs::msg::Image> msg_imgs;

  // Camera sensor info buffer from simulator
  gazebo::msgs::CameraSensor m_pbTmpBufCameraSensorInfo;

  // Camera info publishers.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubCameraInfos;

  // Camera info managers
  std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> cameraInfoManager;

  // Image publisher
  std::vector<image_transport::Publisher> pubImages;
};

#endif