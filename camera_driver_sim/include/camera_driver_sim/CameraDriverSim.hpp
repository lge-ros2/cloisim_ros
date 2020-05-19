/**
 *  @file   CameraDriverSim.hpp
 *  @date   2020-05-13
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
#ifndef _CameraDriverSim_H_
#define _CameraDriverSim_H_

#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <protobuf/image_stamped.pb.h>
#include "sim_bridge/sim_bridge.hpp"

class CameraDriverSim : public rclcpp::Node
{
public:
  static std::string GetEncondingType(const uint32_t pixel_format);

  CameraDriverSim();
  virtual ~CameraDriverSim();

private:
  void Start();
  void Stop();
  void ReadProc();
  void UpdateImage();
  void UpdateStaticTF(const rclcpp::Time timestamp);

private:
  SimBridge *m_pSimBridge;
  bool m_bRun;
  bool m_bIntensity;

  std::string robot_name_;
  std::string part_name_;
  std::string topic_name_;
  std::string frame_name_;

  geometry_msgs::msg::TransformStamped camera_tf;

  std::thread m_thread;
  std::string m_hashKeySub;

  gazebo::msgs::ImageStamped m_pbBuf;
  sensor_msgs::msg::Image msg_img;

  rclcpp::Node::SharedPtr node_handle;
  rclcpp::Time m_simTime;

  // Image publisher
  image_transport::Publisher pubImage;

  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // Static TF
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

};
#endif