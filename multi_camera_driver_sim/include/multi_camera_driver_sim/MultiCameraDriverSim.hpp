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

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <protobuf/images_stamped.pb.h>
#include "sim_bridge/sim_bridge.hpp"

class MultiCameraDriverSim : public rclcpp::Node
{
public:
  MultiCameraDriverSim();
  virtual ~MultiCameraDriverSim();

public:
  static std::string GetEncondingType(const uint32_t pixel_format);

private:
  void Start();
  void Stop();
  void UpdateImage();
  void UpdateStaticTF(const rclcpp::Time timestamp);

private:
  std::string m_hashKeySub;

  SimBridge *m_pSimBridge;

  bool m_bRunThread;
  std::thread m_thread;

  bool m_bIntensity;

private:
  std::vector<geometry_msgs::msg::TransformStamped> camera_tf_list;

  gazebo::msgs::ImagesStamped m_pbBuf;

  sensor_msgs::msg::Image msg_img;

  rclcpp::Time m_simTime;

private:
  rclcpp::Node::SharedPtr node_handle;

  rclcpp::TimerBase::SharedPtr timer;

  std::vector<image_transport::Publisher> image_pubs_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};
#endif