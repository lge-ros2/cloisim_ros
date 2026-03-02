/**
 *  @file   segmentation_camera.cpp
 *  @date   2024-03-01
 *  @author hyunseok Yang
 *  @brief
 *        ROS2 Segmentation Camera class for simulator
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *      SPDX-License-Identifier: MIT
 */

#include "cloisim_ros_camera/segmentation_camera.hpp"

#include <cstring>

using namespace std::placeholders;
using string = std::string;

namespace cloisim_ros
{
SegmentationCamera::SegmentationCamera(
  const rclcpp::NodeOptions & options_, const string node_name, const string namespace_)
: CameraBase(options_, node_name, namespace_)
{
  Start();
}

SegmentationCamera::SegmentationCamera(const string node_name, const string namespace_)
: SegmentationCamera(rclcpp::NodeOptions(), node_name, namespace_)
{
}

SegmentationCamera::~SegmentationCamera()
{
  // DBG_SIM_INFO("Delete %s", typeid(this).name());
  Stop();
}

void SegmentationCamera::InitializeCameraData()
{
  uint16_t portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");

  auto data_bridge_ptr = CreateBridge();
  if (data_bridge_ptr != nullptr) {
    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddBridgeReceiveWorker(data_bridge_ptr, bind(&SegmentationCamera::PublishData, this, _1, _2));
  }

  pub_labelinfo_ = create_publisher<vision_msgs::msg::LabelInfo>(
    topic_base_name_ + "/label_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  LOG_I(this, typeid(this).name() << "hashKey: data(" << hashKeyData << ")");
}

void SegmentationCamera::PublishData(const void* buffer, int bufferLength)
{
  // Try raw binary format first (RAWS magic)
  if (bufferLength >= static_cast<int>(sizeof(RawImageHeader))) {
    uint32_t magic;
    std::memcpy(&magic, buffer, sizeof(uint32_t));

    if (magic == MAGIC_RAW_SEGMENTATION) {
      const auto* hdr = reinterpret_cast<const RawImageHeader*>(buffer);
      const auto pixelDataLen = static_cast<size_t>(hdr->height) * hdr->step;
      const auto pixelEnd = sizeof(RawImageHeader) + pixelDataLen;

      if (bufferLength >= static_cast<int>(pixelEnd + 4)) {
        // Set timestamp
        cloisim::msgs::Time time_msg;
        time_msg.set_sec(hdr->sec);
        time_msg.set_nsec(hdr->nsec);
        SetTime(time_msg);

        // Parse class map suffix: [class_count:4][per-class: class_id:4 + name_len:2 + name...]
        const auto* suffix = static_cast<const uint8_t*>(buffer) + pixelEnd;
        const auto suffixLen = bufferLength - static_cast<int>(pixelEnd);

        uint32_t class_count;
        std::memcpy(&class_count, suffix, 4);

        vision_msgs::msg::LabelInfo msg_label_info;
        vision_msgs::msg::VisionClass msg_vision_class;
        msg_label_info.header.stamp = GetTime();

        size_t offset = 4;
        for (uint32_t i = 0; i < class_count && offset + 6 <= static_cast<size_t>(suffixLen); i++) {
          uint32_t class_id;
          uint16_t name_len;
          std::memcpy(&class_id, suffix + offset, 4);
          std::memcpy(&name_len, suffix + offset + 4, 2);
          offset += 6;

          std::string class_name;
          if (name_len > 0 && offset + name_len <= static_cast<size_t>(suffixLen)) {
            class_name.assign(reinterpret_cast<const char*>(suffix + offset), name_len);
            offset += name_len;
          }

          msg_vision_class.class_id = class_id;
          msg_vision_class.class_name = class_name;
          msg_label_info.class_map.push_back(msg_vision_class);
        }

        msg_label_info.threshold = 1;
        pub_labelinfo_->publish(msg_label_info);

        // Publish image via raw path
        const auto* pixelData = static_cast<const uint8_t*>(buffer) + sizeof(RawImageHeader);
        CameraBase::PublishRawImage(*hdr, pixelData, pixelDataLen);
        return;
      }
    }
  }

  // Fallback: protobuf deserialization
  if (!pb_seg_.ParseFromArray(buffer, bufferLength)) {
    DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
    return;
  }

  SetTime(pb_seg_.image_stamped().time());

  vision_msgs::msg::LabelInfo msg_label_info;
  vision_msgs::msg::VisionClass msg_vision_class;
  msg_label_info.header.stamp = GetTime();

  for (auto i = 0; i < pb_seg_.class_map_size(); i++) {
    auto & class_map = pb_seg_.class_map(i);
    msg_vision_class.class_id = class_map.class_id();
    msg_vision_class.class_name = class_map.class_name();

    msg_label_info.class_map.push_back(msg_vision_class);
  }

  msg_label_info.threshold = 1;

  pub_labelinfo_->publish(msg_label_info);

  // send camera image
  CameraBase::PublishData(pb_seg_.image_stamped());
}
}  // namespace cloisim_ros
