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

#include <driver_sim/driver_sim.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <protobuf/images_stamped.pb.h>
#include <protobuf/camerasensor.pb.h>

class MultiCameraDriverSim : public DriverSim
{
public:
  explicit MultiCameraDriverSim(const std::string node_name = "multi_camera_driver_sim");
  virtual ~MultiCameraDriverSim();

private:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index) override;

  void GetRos2FramesId(SimBridge* const pSimBridge);

private:
  std::string multicamera_name_;

  uint16_t portData_;
  std::string m_hashKeySub;

  std::vector<std::string> frame_id_;

  // buffer from simulation
  gazebo::msgs::ImagesStamped m_pbBuf;

  // message for ROS2 communictaion
  std::map<int, sensor_msgs::msg::Image> msg_imgs_;

  // Camera info publishers.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubCamerasInfo;

  // Camera info managers
  std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> cameraInfoManager;

  // Image publisher
  std::vector<image_transport::Publisher> pubImages;
};
#endif
