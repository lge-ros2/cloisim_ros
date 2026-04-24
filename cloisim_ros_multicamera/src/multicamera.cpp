/**
 *  @file   multicamera.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Multi Camera class for simulator
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include <cloisim_msgs/param.pb.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cloisim_ros_multicamera/multicamera.hpp"
#include <cloisim_ros_base/camera_helper.hpp>
#include <sensor_msgs/fill_image.hpp>

#include <cstdint>
#include <cstring>

using namespace std::literals::chrono_literals;
using string = std::string;

namespace cloisim_ros
{

MultiCamera::MultiCamera(
  const rclcpp::NodeOptions & options_, const string node_name, const std::string namespace_)
: Base(node_name, namespace_, options_)
{
  Start();
}

MultiCamera::MultiCamera(const std::string namespace_)
: MultiCamera(rclcpp::NodeOptions(), "cloisim_ros_multicamera", namespace_)
{
}

MultiCamera::~MultiCamera() {Stop();}

void MultiCamera::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  LOG_I(this, "hashKey: data(" << hashKeyData << ") info(" << hashKeyInfo << ")");

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr) {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    image_transport::ImageTransport it(GetNode());
    for (auto frame_id : frame_id_list_) {
      auto parent_frame_id = std::string("base_link");
      auto transform_pose = GetObjectTransform(info_bridge_ptr, frame_id, parent_frame_id);
      transform_pose.set_name(GetPartsName() + "_" + frame_id);
      SetStaticTf2(transform_pose, parent_frame_id);

      // Image publisher
      const auto topic_base_name_ = GetPartsName() + "/" + frame_id;
      // DBG_SIM_INFO("topic_base_name: %s", topic_base_name_.c_str());

      sensor_msgs::msg::Image msg_img;
      msg_img.header.frame_id = frame_id;

      msg_imgs_[msg_imgs_.size()] = msg_img;

      pubs_.push_back(it.advertiseCamera(topic_base_name_ + "/image_raw", 1));

      camera_info_manager_.push_back(
        std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get()));
      const auto camSensorMsg = GetCameraSensorMessage(info_bridge_ptr, frame_id);
      SetCameraInfoInManager(camera_info_manager_.back(), camSensorMsg, frame_id);
    }

    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(
      data_bridge_ptr,
        bind(&MultiCamera::PublishData, this, std::placeholders::_1, std::placeholders::_2));
  }
}

void MultiCamera::Deinitialize()
{
  for (auto & pub_ : pubs_) {
    pub_.shutdown();
  }
  pubs_.clear();
}

void MultiCamera::PublishRawData(const void * buffer, const int bufferLength)
{
  const auto * sharedHdr = reinterpret_cast<const RawMultiImageHeader *>(buffer);
  const uint32_t image_count = sharedHdr->image_count;

  if (static_cast<int>(camera_info_manager_.size()) != static_cast<int>(image_count)) {
    DBG_SIM_ERR(
      "camera_info_manager is not ready for multi-camera %zu != %u",
      camera_info_manager_.size(), image_count);
    return;
  }

  cloisim::msgs::Time time_msg;
  time_msg.set_sec(sharedHdr->sec);
  time_msg.set_nsec(sharedHdr->nsec);
  SetTime(time_msg);

  const auto * ptr = static_cast<const uint8_t *>(buffer) + sizeof(RawMultiImageHeader);
  const auto * end = static_cast<const uint8_t *>(buffer) + bufferLength;

  for (uint32_t i = 0; i < image_count; i++) {
    if (ptr + sizeof(RawImageSubHeader) > end) {
      DBG_SIM_ERR("RAWM: sub-header overflow at image %u", i);
      return;
    }

    const auto * subHdr = reinterpret_cast<const RawImageSubHeader *>(ptr);
    ptr += sizeof(RawImageSubHeader);

    const auto pixelDataLen = static_cast<size_t>(subHdr->height) * subHdr->step;
    if (ptr + pixelDataLen > end) {
      DBG_SIM_ERR("RAWM: pixel data overflow at image %u", i);
      return;
    }

    const auto encoding_arg = GetImageEncondingType(subHdr->pixel_format);
    auto const msg_img = &msg_imgs_[i];
    msg_img->header.stamp = GetTime();
    msg_img->encoding = encoding_arg;
    msg_img->width = subHdr->width;
    msg_img->height = subHdr->height;
    msg_img->step = subHdr->step;
    msg_img->is_bigendian = false;

    if (msg_img->data.size() != pixelDataLen) {
      msg_img->data.resize(pixelDataLen);
    }
    std::memcpy(msg_img->data.data(), ptr, pixelDataLen);
    ptr += pixelDataLen;

    auto camera_info_msg = camera_info_manager_[i]->getCameraInfo();
    camera_info_msg.header.stamp = GetTime();
    pubs_.at(i).publish(*msg_img, camera_info_msg);
  }
}

void MultiCamera::PublishData(const void * buffer, int bufferLength)
{
  if (pubs_.size() == 0) {return;}

  // Try raw binary format first (RAWM magic)
  if (bufferLength >= static_cast<int>(sizeof(RawMultiImageHeader))) {
    uint32_t magic;
    std::memcpy(&magic, buffer, sizeof(uint32_t));

    if (magic == MAGIC_RAW_MULTI_IMAGE) {
      PublishRawData(buffer, bufferLength);
      return;
    }
  }

  // Fallback: protobuf deserialization
  if (!pb_buf_.ParseFromArray(buffer, bufferLength)) {
    LOG_E(this, "##Parsing error, size=" << bufferLength);
    return;
  }

  SetTime(pb_buf_.time());

  if (static_cast<int>(camera_info_manager_.size()) != pb_buf_.image_size()) {
    DBG_SIM_ERR(
      "camera_info_manager is not ready for multi-camera %d != %d", camera_info_manager_.size(),
      pb_buf_.image_size());
    return;
  }

  for (auto i = 0; i < pb_buf_.image_size(); i++) {
    auto img = &pb_buf_.image(i);
    const auto encoding_arg = GetImageEncondingType(img->pixel_format_type());
    const uint32_t cols_arg = img->width();
    const uint32_t rows_arg = img->height();
    const uint32_t step_arg = img->step();

    auto const msg_img = &msg_imgs_[i];
    msg_img->header.stamp = GetTime();
    sensor_msgs::fillImage(
      *msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
      reinterpret_cast<const void *>(img->data().data()));

    // Publish camera info
    auto camera_info_msg = camera_info_manager_[i]->getCameraInfo();
    camera_info_msg.header.stamp = GetTime();

    pubs_.at(i).publish(*msg_img, camera_info_msg);
  }
}

}  // namespace cloisim_ros
