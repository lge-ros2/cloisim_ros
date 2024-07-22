/**
 *  @file   realsense.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 realsense class for simulator
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/image_stamped.pb.h>
#include <cloisim_msgs/imu.pb.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_realsense/realsense.hpp"
#include <cloisim_ros_base/camera_helper.hpp>
#include <cloisim_ros_base/helper.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;
using namespace std::literals::chrono_literals;
using string = std::string;

namespace cloisim_ros
{

RealSense::RealSense(
    const rclcpp::NodeOptions& options_,
    const string node_name,
    const string namespace_)
    : Base(node_name, namespace_, options_)
{
  Start();
}

RealSense::RealSense(const string namespace_)
    : RealSense(rclcpp::NodeOptions(), "cloisim_ros_realsense", namespace_)
{
}

RealSense::~RealSense()
{
  // DBG_SIM_INFO("Delete");
  Stop();
}

void RealSense::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyInfo = GetTargetHashKey("Info");

  auto info_bridge_ptr = CreateBridge();
  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);
    GetActivatedModules(info_bridge_ptr);

    auto transform_pose = GetObjectTransform(info_bridge_ptr);
    const auto header_frame_id = GetPartsName() + "_link";
    transform_pose.set_name(header_frame_id);
    SetStaticTf2(transform_pose);

    GetRos2Parameter(info_bridge_ptr);
  }

  uint16_t portData;
  for (const auto& module : activated_modules_)
  {
    const auto module_type = std::get<0>(module);
    const auto module_name = std::get<1>(module);

    get_parameter_or("bridge." + module_name + "Data", portData, uint16_t(0));
    get_parameter_or("bridge." + module_name + "Info", portInfo, uint16_t(0));

    const auto hashKeyData = GetTargetHashKey(module_name + "Data");
    const auto hashKeyInfo = GetTargetHashKey(module_name + "Info");

    DBG_SIM_INFO("module: %s, hashKey: data(%s), info(%s)",
                 module_name.c_str(), hashKeyData.c_str(), hashKeyInfo.c_str());

    auto info_bridge_info = CreateBridge();
    if (info_bridge_info->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo))
    {
      auto data_bridge_ptr = CreateBridge();
      data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);

      if (module_type.compare("camera") == 0)
      {
        InitializeCam(module_name, info_bridge_info, data_bridge_ptr);
      }
      else if (module_type.compare("imu") == 0)
      {
        InitializeImu(info_bridge_info, data_bridge_ptr);
      }
      else
      {
        DBG_SIM_ERR("Unknown module type: %s name: %s",
                    module_type.c_str(), module_name.c_str());
      }
    }
  }
}

void RealSense::InitializeCam(
    const string module_name,
    zmq::Bridge* const info_ptr,
    zmq::Bridge* const data_ptr)
{
  if (info_ptr != nullptr)
  {
    auto transform_pose = GetObjectTransform(info_ptr, module_name);
    const auto header_frame_id = GetPartsName() + "_link";
    const auto child_frame_id = GetPartsName() + "_camera_" + module_name + "_frame";
    transform_pose.set_name(child_frame_id);
    SetStaticTf2(transform_pose, header_frame_id);

    const auto camInfoManager =
        std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());

    const auto camSensorMsg = GetCameraSensorMessage(info_ptr);
    SetCameraInfoInManager(camInfoManager, camSensorMsg, module_name);

    camera_info_managers_[data_ptr] = camInfoManager;
  }

  const auto topic_name = (module_name == "depth") ? "image_rect_raw" : "image_raw";

  image_transport::ImageTransport it(GetNode());

  const auto topic_base_name = GetPartsName() + "/" + module_name;
  pubs_[data_ptr] = it.advertiseCamera(topic_base_name + "/" + topic_name, 1);

  sensor_msgs::msg::Image msg_img;
  msg_img.header.frame_id = module_name;

  msg_imgs_[data_ptr] = msg_img;

  if (data_ptr != nullptr)
  {
    AddPublisherThread(data_ptr, bind(&RealSense::PublishImgData, this, data_ptr, _1));
  }
}

void RealSense::InitializeImu(
    zmq::Bridge* const info_ptr,
    zmq::Bridge* const data_ptr)
{
  // Get frame for message
  const auto frame_id = GetFrameId("imu_link");
  msg_imu_.header.frame_id = frame_id;

  if (info_ptr != nullptr)
  {
    auto transform_pose = GetObjectTransform(info_ptr);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose);
  }

  // ROS2 Publisher
  const auto topic_base_name = GetPartsName() + "/" + topic_name_;
  pub_imu_ =
      this->create_publisher<sensor_msgs::msg::Imu>(topic_base_name, rclcpp::SensorDataQoS());

  if (data_ptr != nullptr)
  {
    AddPublisherThread(data_ptr, bind(&RealSense::PublishImuData, this, _1));
  }
}

void RealSense::Deinitialize()
{
  for (auto& pub : pubs_)
    pub.second.shutdown();
  pubs_.clear();
}

void RealSense::GetActivatedModules(zmq::Bridge* const bridge_ptr)
{
  if (bridge_ptr == nullptr)
  {
    return;
  }

  string moduleListStr;
  const auto reply = RequestReplyMessage(bridge_ptr, "request_module_list");

  const auto reply_size = reply.ByteSizeLong();
  if (reply_size <= 0)
  {
    DBG_SIM_ERR("Failed to get activated module info, length(%ld)", reply_size);
  }
  else
  {
    if (reply.IsInitialized() &&
        reply.name() == "activated_modules")
    {
      for (auto i = 0; i < reply.children_size(); i++)
      {
        const auto param = reply.children(i);
        if (param.name() == "module" && param.children_size() == 2)
        {
          const auto type = param.children(0);
          const auto name = param.children(1);
          if (type.has_value() &&
              type.value().type() == cloisim::msgs::Any_ValueType_STRING &&
              !type.value().string_value().empty() &&
              name.has_value() &&
              name.value().type() == cloisim::msgs::Any_ValueType_STRING &&
              !name.value().string_value().empty())
          {
            const auto tuple_module =
                std::make_tuple(type.value().string_value(), name.value().string_value());
            activated_modules_.push_back(tuple_module);
            moduleListStr.append(std::get<1>(tuple_module) +
                                 "(" + std::get<0>(tuple_module) + "), ");
          }
        }
      }
    }
  }

  DBG_SIM_INFO("activated_modules: %s", moduleListStr.c_str());
}

void RealSense::PublishImgData(
    const zmq::Bridge* const bridge_ptr,
    const std::string& buffer)
{
  cloisim::msgs::ImageStamped pb_buf_;
  if (!pb_buf_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.time());

  auto const msg_img = &msg_imgs_[bridge_ptr];
  msg_img->header.stamp = GetTime();

  const auto encoding_arg = GetImageEncondingType(pb_buf_.image().pixel_format());
  const uint32_t cols_arg = pb_buf_.image().width();
  const uint32_t rows_arg = pb_buf_.image().height();
  const uint32_t step_arg = pb_buf_.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(*msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void*>(pb_buf_.image().data().data()));

  // Publish camera info
  auto camera_info_msg = camera_info_managers_[bridge_ptr]->getCameraInfo();
  camera_info_msg.header.stamp = GetTime();

  pubs_[bridge_ptr].publish(*msg_img, camera_info_msg);
}

void RealSense::PublishImuData(const string& buffer)
{
  cloisim::msgs::IMU pb_buf_;
  if (!pb_buf_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.stamp());

  // Fill message with latest sensor data
  msg_imu_.header.stamp = GetTime();

  SetQuaternionMessageToGeometry(pb_buf_.orientation(), msg_imu_.orientation);
  SetVector3MessageToGeometry(pb_buf_.angular_velocity(), msg_imu_.angular_velocity);
  SetVector3MessageToGeometry(pb_buf_.linear_acceleration(), msg_imu_.linear_acceleration);

  std::fill(begin(msg_imu_.orientation_covariance),
            end(msg_imu_.orientation_covariance), 0.0);
  std::fill(begin(msg_imu_.angular_velocity_covariance),
            end(msg_imu_.angular_velocity_covariance), 0.0);
  std::fill(begin(msg_imu_.linear_acceleration_covariance),
            end(msg_imu_.linear_acceleration_covariance), 0.0);

  pub_imu_->publish(msg_imu_);
}

}  // namespace cloisim_ros
