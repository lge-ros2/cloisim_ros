/**
 *  @file   ground_truth.hpp
 *  @date   2021-05-16
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 node for controlling unity simulation
 *  @remark
 *        Gazebonity
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLOISIM_ROS_GROUNDTRUTH_HPP_
#define _CLOISIM_ROS_GROUNDTRUTH_HPP_

#include <cloisim_ros_base/base.hpp>
#include <cloisim_msgs/perception_v.pb.h>
#include <perception_msgs/msg/object_array.hpp>

namespace cloisim_ros
{
  class GroundTruth : public Base
  {
  public:
    explicit GroundTruth(const rclcpp::NodeOptions &options_, const std::string node_name_);
    explicit GroundTruth();
    virtual ~GroundTruth();

  private:
    virtual void Initialize() override;
    virtual void Deinitialize() override;
    virtual void UpdatePublishingData(const std::string &buffer) override;

    void UpdatePerceptionData();

  private:
    cloisim::msgs::Perception_V pbBuf;

    perception_msgs::msg::ObjectArray msg;

    rclcpp::Publisher<perception_msgs::msg::ObjectArray>::SharedPtr pub;
  };
}
#endif