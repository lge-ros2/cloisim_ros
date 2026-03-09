/**
 *  @file   camera_base.hpp
 *  @date   2024-03-08
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CameraBase class
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2024 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef CLOISIM_ROS_CAMERA__CAMERA_BASE_HPP_
#define CLOISIM_ROS_CAMERA__CAMERA_BASE_HPP_

#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/image_stamped.pb.h>
#include <cloisim_msgs/pose.pb.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cloisim_ros_base/base.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace cloisim_ros
{

// Magic numbers for raw binary image transport (little-endian on wire)
static constexpr uint32_t MAGIC_RAW_IMAGE = 0x52415749u;         // "RAWI"
static constexpr uint32_t MAGIC_RAW_SEGMENTATION = 0x52415753u;  // "RAWS"

// 28-byte fixed header for RAWI / RAWS
#pragma pack(push, 1)
struct RawImageHeader
{
  uint32_t magic;
  int32_t  sec;
  int32_t  nsec;
  uint32_t width;
  uint32_t height;
  uint32_t pixel_format;
  uint32_t step;
};
#pragma pack(pop)
static_assert(sizeof(RawImageHeader) == 28, "RawImageHeader must be 28 bytes");

// 16-byte per-image sub-header inside RAWM
#pragma pack(push, 1)
struct RawImageSubHeader
{
  uint32_t width;
  uint32_t height;
  uint32_t pixel_format;
  uint32_t step;
};
#pragma pack(pop)
static_assert(sizeof(RawImageSubHeader) == 16, "RawImageSubHeader must be 16 bytes");

class CameraBase : public Base
{
public:
  explicit CameraBase(
    const rclcpp::NodeOptions & options_, const std::string node_name,
    const std::string namespace_ = "");
  explicit CameraBase(const std::string node_name, const std::string namespace_ = "");
  virtual ~CameraBase();

protected:
  void Initialize() override;
  void Deinitialize() override;

  void InitializeCameraInfo();

  void InitializeCameraPublish();
  virtual void InitializeCameraData();

protected:
  void PublishData(const void * buffer, int bufferLength);
  void PublishData(const cloisim::msgs::ImageStamped & pb_msg);

  /// Publish from a raw RAWI buffer — no protobuf parse. Returns false if not a raw buffer.
  bool TryPublishRawImage(const void * buffer, int bufferLength);

  /// Fill msg_img_ and publish from a RawImageHeader + raw pixel pointer.
  void PublishRawImage(const RawImageHeader & hdr, const void * pixelData, size_t pixelDataLen);

protected:
  std::string frame_id_;
  std::string optical_frame_id_;
  std::string topic_base_name_;

  // image buffer from simulator
  cloisim::msgs::ImageStamped pb_img_;

  // message for ROS2 communictaion
  sensor_msgs::msg::Image msg_img_;

  // Image publisher
  image_transport::CameraPublisher pub_;

  // Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};
}  // namespace cloisim_ros
#endif  // CLOISIM_ROS_CAMERA__CAMERA_BASE_HPP_
