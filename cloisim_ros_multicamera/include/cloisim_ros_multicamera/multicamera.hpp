/**
 *  @file   multicamera.hpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Multi Camera class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _CLOISIM_ROS_MULTICAMERA_HPP_
#define _CLOISIM_ROS_MULTICAMERA_HPP_

#include <cloisim_ros_base/base.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cloisim_msgs/images_stamped.pb.h>
#include <cloisim_msgs/camerasensor.pb.h>

namespace cloisim_ros
{
  class MultiCamera : public Base
  {
  public:
    explicit MultiCamera(const rclcpp::NodeOptions &options_, const std::string node_name_, const std::string namespace_ = "");
    explicit MultiCamera(const std::string namespace_ = "");
    virtual ~MultiCamera();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdateData(const uint bridge_index) override;

    void GetRos2FramesId(zmq::Bridge *const pBridge);

  private:
    std::string multicamera_name_;

    std::string hashKeySub_;

    std::vector<std::string> frame_id_;

    // buffer from simulation
    cloisim::msgs::ImagesStamped pbBuf_;

    // message for ROS2 communictaion
    std::map<int, sensor_msgs::msg::Image> msgImgs_;

    // Camera info managers
    std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> cameraInfoManager;

    // Image publisher
    std::vector<image_transport::CameraPublisher> pubImages_;
  };
}
#endif
