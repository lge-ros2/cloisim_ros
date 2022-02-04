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
#include <sensor_msgs/msg/imu.hpp>
#include <tuple>

namespace cloisim_ros
{
  class RealSense : public Base
  {
  public:
    explicit RealSense(const rclcpp::NodeOptions &options_, const std::string node_name, const std::string namespace_ = "");
    explicit RealSense(const std::string namespace_ = "");
    virtual ~RealSense();

  private:
    void Initialize() override;
    void Deinitialize() override;

  private:
    void InitializeCam(const std::string module_name, zmq::Bridge* const info_ptr, zmq::Bridge* const data_ptr);
    void InitializeImu(zmq::Bridge* const info_ptr, zmq::Bridge* const data_ptr);
    void PublishImgData(const zmq::Bridge* const bridge_ptr, const std::string &buffer);
    void PublishImuData(const std::string &buffer);
    void GetActivatedModules(zmq::Bridge *const bridge_ptr);

  private:
    std::vector<std::tuple<std::string, std::string>> activated_modules_;

    // message for ROS2 communictaion
    std::map<const zmq::Bridge* const, sensor_msgs::msg::Image> msg_imgs_;

    // Camera info managers
    std::map<const zmq::Bridge* const, std::shared_ptr<camera_info_manager::CameraInfoManager>> camera_info_managers_;

    // Image publisher
    std::map<const zmq::Bridge* const, image_transport::CameraPublisher> pubs_;

    // IMU msgs
    sensor_msgs::msg::Imu msg_imu_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  };
}
#endif