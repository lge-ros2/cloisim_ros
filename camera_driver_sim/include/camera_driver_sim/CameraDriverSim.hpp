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

#include <driver_sim/driver_sim.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <protobuf/image_stamped.pb.h>
#include <protobuf/camerasensor.pb.h>
#include <protobuf/pose.pb.h>

class CameraDriverSim : public DriverSim
{
public:
  explicit CameraDriverSim(const std::string node_name = "camera_driver_sim");
  virtual ~CameraDriverSim();

protected:
  virtual void Initialize() override;
  virtual void Deinitialize() override;
  virtual void UpdateData(const uint bridge_index = 0) override;
  virtual void SetupStaticTf2Message(const gazebo::msgs::Pose transform, const std::string frame_id) override;

private:
  void GetCameraSensorMessage(SimBridge* const pSimBridge);
  void InitializeCameraInfoMessage(const std::string frame_id);

private:
  // key for connection
  uint16_t portData_;
  std::string m_hashKeySub;

  // image buffer from simulator
  gazebo::msgs::ImageStamped m_pbImgBuf;

  // message for ROS2 communictaion
  sensor_msgs::msg::Image msg_img;

  // Image publisher
  image_transport::Publisher pubImage;

  // Camera sensor info buffer from simulator
  gazebo::msgs::CameraSensor m_pbBufCameraSensorInfo;

  // Camera info publisher
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubCameraInfo{nullptr};

  // Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;
};

#endif