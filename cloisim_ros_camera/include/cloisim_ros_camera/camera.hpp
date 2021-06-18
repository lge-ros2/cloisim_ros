/**
 *  @file   camera.hpp
 *  @date   2021-01-14
 *  @author Sungkyu Kang
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Camera class for cloisim
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _CLOISIM_ROS_CAMERA_HPP_
#define _CLOISIM_ROS_CAMERA_HPP_

#include <cloisim_ros_base/base.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cloisim_msgs/image_stamped.pb.h>
#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/pose.pb.h>

namespace cloisim_ros
{
  class Camera : public Base
  {
  public:
    explicit Camera(const rclcpp::NodeOptions &options_, const std::string node_name = "cloisim_ros_camera", const std::string namespace_ = "");
    explicit Camera(const std::string node_name = "cloisim_ros_camera", const std::string namespace_ = "");
    virtual ~Camera();

  protected:
    void Initialize() override;
    void Deinitialize() override;

  protected:
    void PublishData(const std::string &buffer);

  private:
    // image buffer from simulatornode_name_
    cloisim::msgs::ImageStamped pb_img_;

    // message for ROS2 communictaion
    sensor_msgs::msg::Image msg_img_;

    // Image publisher
    image_transport::CameraPublisher pub_;

    // Camera info manager
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  };
}
#endif