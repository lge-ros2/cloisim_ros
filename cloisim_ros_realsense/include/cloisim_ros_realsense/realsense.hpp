/**
 *  @file   realsense.hpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 realsense class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#ifndef _CLOISIM_ROS_REALSENSE_HPP_
#define _CLOISIM_ROS_REALSENSE_HPP_

#include <cloisim_ros_base/base.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cloisim_msgs/image_stamped.pb.h>
#include <cloisim_msgs/camerasensor.pb.h>

namespace cloisim_ros
{
  class RealSense : public Base
  {
  public:
    explicit RealSense(const rclcpp::NodeOptions &options_, const std::string node_name_, const std::string namespace_ = "");
    explicit RealSense(const std::string namespace_ = "");
    virtual ~RealSense();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdateData(const uint bridge_index) override;

    void GetActivatedModules(zmq::Bridge *const pBridge);

  private:
    cloisim::msgs::CameraSensor m_pbTmpBufCameraSensorInfo;

    std::vector<std::string> module_list_;
    std::vector<std::thread> threads_;

    // key for connection
    std::map<int, uint16_t> dataPortMap_;
    std::map<int, std::string> hashKeySubs_;

    // message for ROS2 communictaion
    std::map<int, sensor_msgs::msg::Image> msgImgs_;

    // Camera info managers
    std::map<int, std::shared_ptr<camera_info_manager::CameraInfoManager>> cameraInfoManager_;

    // Image publisher
    std::map<int, image_transport::CameraPublisher> pubImages_;
  };
}
#endif