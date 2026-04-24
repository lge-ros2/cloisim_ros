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
#include <cloisim_msgs/image.pb.h>
#include <cloisim_msgs/imu.pb.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_realsense/realsense.hpp"
#include <cloisim_ros_base/camera_helper.hpp>
#include <cloisim_ros_base/param_helper.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;
using namespace std::literals::chrono_literals;

namespace cloisim_ros
{

RealSense::RealSense(
  const rclcpp::NodeOptions & options_, const std::string node_name, const std::string namespace_)
: Base(node_name, namespace_, options_)
  , header_frame_id_("realsense_link")
{
  topic_name_ = "_";
  Start();
}

RealSense::RealSense(const std::string namespace_)
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
  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);
    GetActivatedModules(info_bridge_ptr);

    auto parent_frame_id = std::string("base_link");
    auto transform_pose = GetObjectTransform(info_bridge_ptr, parent_frame_id);
    header_frame_id_ = GetPartsName() + "_link";
    transform_pose.set_name(header_frame_id_);
    SetStaticTf2(transform_pose, parent_frame_id);

    GetRos2Parameter(info_bridge_ptr);
  }

  uint16_t portData;
  for (const auto & module : activated_modules_) {
    const auto module_type = std::get<0>(module);
    const auto module_name = std::get<1>(module);

    get_parameter_or("bridge." + module_name + "Data", portData, uint16_t(0));
    get_parameter_or("bridge." + module_name + "Info", portInfo, uint16_t(0));

    const auto hashKeyData = GetTargetHashKey(module_name + "Data");
    const auto hashKeyInfo = GetTargetHashKey(module_name + "Info");
    LOG_I(this,
        "module=" << module_name << "hashKey: data(" << hashKeyData << ") info(" << hashKeyInfo <<
        ")");

    auto info_bridge_info = CreateBridge();
    if (info_bridge_info->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo)) {
      auto data_bridge_ptr = CreateBridge();
      data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);

      if (module_type.compare("camera") == 0) {
        InitializeCam(module_name, info_bridge_info, data_bridge_ptr);
      } else if (module_type.compare("imu") == 0) {
        InitializeImu(info_bridge_info, data_bridge_ptr);
      } else {
        LOG_E(this, "Unknown module type=" << module_type << " name=" << module_name);
      }
    }
  }
}

void RealSense::InitializeCam(
  const std::string module_name, zmq::Bridge * const info_ptr, zmq::Bridge * const data_ptr)
{
  if (info_ptr != nullptr) {
    auto transform_pose = GetTargetObjectTransform(info_ptr, module_name);
    const auto child_frame_id = GetPartsName() + "_camera_" + module_name + "_frame";
    transform_pose.set_name(child_frame_id);
    SetStaticTf2(transform_pose, header_frame_id_);

    const auto camInfoManager =
      std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());

    const auto camSensorMsg = GetCameraSensorMessage(info_ptr);
    SetCameraInfoInManager(camInfoManager, camSensorMsg, module_name);

    camera_info_managers_[data_ptr] = camInfoManager;
  }

  const auto topic_name = (module_name == "depth") ? "image_rect_raw" : "image_raw";

  image_transport::ImageTransport it(GetNode());

  const auto topic_base_name = GetPartsName() + "/" + module_name;
  const auto full_image_topic = topic_base_name + "/" + topic_name;

  // Disable the "compressed" image_transport plugin for depth-related modules.
  // It does not support depth encodings (16UC1, 32FC1) and spams errors.
  if (module_name.find("depth") != std::string::npos) {
    auto param_name = full_image_topic;
    std::replace(param_name.begin(), param_name.end(), '/', '.');
    param_name += ".enable_pub_plugins";
    try {
      this->declare_parameter(
        param_name,
        std::vector<std::string>{
          "image_transport/raw",
          "image_transport/compressedDepth",
          "image_transport/theora",
          "image_transport/zstd"});
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    }
  }

  pubs_[data_ptr] = it.advertiseCamera(full_image_topic, 1);

  sensor_msgs::msg::Image msg_img;
  msg_img.header.frame_id = module_name;

  msg_imgs_[data_ptr] = msg_img;

  if (data_ptr != nullptr) {
    AddBridgeReceiveWorker(data_ptr, bind(&RealSense::PublishImgData, this, data_ptr, _1, _2));
  }
}

void RealSense::InitializeImu(zmq::Bridge * const info_ptr, zmq::Bridge * const data_ptr)
{
  // Get frame for message
  const auto topic_name = "imu";
  const auto frame_id = GetFrameId("imu_link");
  msg_imu_.header.frame_id = frame_id;

  if (info_ptr != nullptr) {
    auto parent_frame_id = std::string("base_link");
    auto transform_pose = GetObjectTransform(info_ptr, parent_frame_id);
    transform_pose.set_name(frame_id);
    SetStaticTf2(transform_pose, parent_frame_id);
  }

  // ROS2 Publisher
  const auto topic_base_name = GetPartsName() + "/" + topic_name;
  pub_imu_ =
    this->create_publisher<sensor_msgs::msg::Imu>(topic_base_name, rclcpp::SensorDataQoS());

  if (data_ptr != nullptr) {
    AddBridgeReceiveWorker(data_ptr, bind(&RealSense::PublishImuData, this, _1, _2));
  }
}

void RealSense::Deinitialize()
{
  for (auto & pub : pubs_) {
    pub.second.shutdown();
  }
  pubs_.clear();
}

void RealSense::GetActivatedModules(zmq::Bridge * const bridge_ptr)
{
  if (bridge_ptr == nullptr) {
    return;
  }

  std::string moduleListStr;
  const auto reply = RequestReplyMessage(bridge_ptr, "request_module_list");

  const auto reply_size = reply.ByteSizeLong();
  if (reply_size <= 0) {
    DBG_SIM_ERR("Failed to get activated module info, length(%ld)", reply_size);
  } else {
    if (reply.IsInitialized() && param::HasKey(reply, "activated_modules")) {
      for (auto i = 0; i < reply.children_size(); i++) {
        const auto & child = reply.children(i);
        const auto child_name = param::GetName(child);
        if (child_name == "module" && child.children_size() == 2) {
          const auto & type = child.children(0);
          const auto & name = child.children(1);
          const auto type_name = param::GetName(type);
          const auto & type_value = param::GetValue(type);
          const auto name_name = param::GetName(name);
          const auto & name_value = param::GetValue(name);
          if (
            param::HasValue(type) &&
            type_value.type() == cloisim::msgs::Any_ValueType_STRING &&
            !type_value.string_value().empty() && param::HasValue(name) &&
            name_value.type() == cloisim::msgs::Any_ValueType_STRING &&
            !name_value.string_value().empty())
          {
            const auto tuple_module =
              std::make_tuple(type_value.string_value(), name_value.string_value());
            activated_modules_.push_back(tuple_module);
            moduleListStr.append(
              std::get<1>(tuple_module) + "(" + std::get<0>(tuple_module) + "), ");
          }
        }
      }
    }
  }

  DBG_SIM_INFO("activated_modules: %s", moduleListStr.c_str());
}

bool RealSense::PublishRawImgData(
  const zmq::Bridge * const bridge_ptr, const void * buffer,
  int bufferLength)
{
  if (bufferLength < static_cast<int>(sizeof(RawImageHeader))) {
    return false;
  }

  uint32_t magic;
  std::memcpy(&magic, buffer, sizeof(uint32_t));

  if (magic != MAGIC_RAW_IMAGE) {
    return false;
  }

  const auto * hdr = reinterpret_cast<const RawImageHeader *>(buffer);
  const auto pixelDataLen = static_cast<size_t>(hdr->height) * hdr->step;
  const auto expectedLen = sizeof(RawImageHeader) + pixelDataLen;

  if (bufferLength < static_cast<int>(expectedLen)) {
    LOG_E(this, "Raw image buffer too short: " << bufferLength << " < " << expectedLen);
    return false;
  }

  const auto * pixelData =
    static_cast<const uint8_t *>(buffer) + sizeof(RawImageHeader);

  cloisim::msgs::Time time_msg;
  time_msg.set_sec(hdr->sec);
  time_msg.set_nsec(hdr->nsec);
  SetTime(time_msg);

  auto const msg_img = &msg_imgs_[bridge_ptr];
  msg_img->header.stamp = GetTime();
  msg_img->encoding = GetImageEncondingType(hdr->pixel_format);
  msg_img->width = hdr->width;
  msg_img->height = hdr->height;
  msg_img->step = hdr->step;
  msg_img->is_bigendian = false;

  if (msg_img->data.size() != pixelDataLen) {
    msg_img->data.resize(pixelDataLen);
  }
  std::memcpy(msg_img->data.data(), pixelData, pixelDataLen);

  auto camera_info_msg = camera_info_managers_[bridge_ptr]->getCameraInfo();
  camera_info_msg.header.stamp = GetTime();
  pubs_[bridge_ptr].publish(*msg_img, camera_info_msg);
  return true;
}

void RealSense::PublishImgData(
  const zmq::Bridge * const bridge_ptr, const void * buffer,
  int bufferLength)
{
  // Fast path: try raw binary format first (RAWI magic)
  if (PublishRawImgData(bridge_ptr, buffer, bufferLength)) {
    return;
  }

  // Fallback: protobuf deserialization (backward-compatible)
  cloisim::msgs::Image pb_buf_;
  if (!pb_buf_.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "##Parsing error, size=" << bufferLength);
    return;
  }

  SetTime(pb_buf_.header().stamp());

  auto const msg_img = &msg_imgs_[bridge_ptr];
  msg_img->header.stamp = GetTime();

  const auto encoding_arg = GetImageEncondingType(pb_buf_.pixel_format_type());
  const uint32_t cols_arg = pb_buf_.width();
  const uint32_t rows_arg = pb_buf_.height();
  const uint32_t step_arg = pb_buf_.step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(
    *msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
    reinterpret_cast<const void *>(pb_buf_.data().data()));

  // Publish camera info
  auto camera_info_msg = camera_info_managers_[bridge_ptr]->getCameraInfo();
  camera_info_msg.header.stamp = GetTime();

  pubs_[bridge_ptr].publish(*msg_img, camera_info_msg);
}

void RealSense::PublishImuData(const void * buffer, int bufferLength)
{
  cloisim::msgs::IMU pb_buf_;
  if (!pb_buf_.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "##Parsing error, size=" << bufferLength);
    return;
  }

  SetTime(pb_buf_.header().stamp());

  // Fill message with latest sensor data
  msg_imu_.header.stamp = GetTime();

  msg::Convert(pb_buf_.orientation(), msg_imu_.orientation);
  msg::Convert(pb_buf_.angular_velocity(), msg_imu_.angular_velocity);
  msg::Convert(pb_buf_.linear_acceleration(), msg_imu_.linear_acceleration);

  std::fill(begin(msg_imu_.orientation_covariance), end(msg_imu_.orientation_covariance), 0.0);
  std::fill(
    begin(msg_imu_.angular_velocity_covariance), end(msg_imu_.angular_velocity_covariance), 0.0);
  std::fill(
    begin(msg_imu_.linear_acceleration_covariance), end(msg_imu_.linear_acceleration_covariance),
    0.0);

  pub_imu_->publish(msg_imu_);
}

}  // namespace cloisim_ros
