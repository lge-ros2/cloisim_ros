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
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <protobuf/image_stamped.pb.h>
#include <protobuf/camerasensor.pb.h>

class RealSenseDriverSim : public DriverSim
{
public:
  explicit RealSenseDriverSim(const std::string node_name = "realsense_driver_sim");
  virtual ~RealSenseDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index) override;

  void GetParameters(SimBridge* const pSimBridge);
  void GetActivatedModules(SimBridge* const pSimBridge);

private:
  double depth_range_min_;
  double depth_range_max_;
  int depth_scale_;

private:
  gazebo::msgs::CameraSensor m_pbTmpBufCameraSensorInfo;

  std::vector<std::string> module_list_;
  std::vector<std::thread> threads_;

  // key for connection
  std::map<int, uint16_t> dataPortMap_;
  std::map<int, std::string> hashKeySubs_;

  // message for ROS2 communictaion
  std::map<int, sensor_msgs::msg::Image> msg_imgs_;

  // // Camera info publishers.
  // std::map<int, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubCameraInfos_;

  // Camera info managers
  std::map<int, std::shared_ptr<camera_info_manager::CameraInfoManager>> cameraInfoManager_;

  // Image publisher
  std::map<int, image_transport::CameraPublisher> pubImages_;
};

#endif