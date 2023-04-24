/**
 *  @file   camera_helper.h
 *  @date   2021-05-16
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 CLOiSim-ROS helper for camera
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLOISIM_ROS_CAMERA_HELPER_H_
#define _CLOISIM_ROS_CAMERA_HELPER_H_

#include <camera_info_manager/camera_info_manager.hpp>
#include <cloisim_ros_bridge_zmq/bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/param.pb.h>

static std::string GetImageEncondingType(const uint32_t pixel_format)
{
  // UNKNOWN_PIXEL_FORMAT = 0, L_INT8, L_INT16,
  // RGB_INT8 = 3, RGBA_INT8, BGRA_INT8, RGB_INT16, RGB_INT32,
  // BGR_INT8 = 8, BGR_INT16, BGR_INT32,
  // R_FLOAT16 = 11, RGB_FLOAT16 = 12, R_FLOAT32 = 13, RGB_FLOAT32 = 14,
  // BAYER_RGGB8 = 15, BAYER_RGGR8, BAYER_GBRG8, BAYER_GRBG8,
  // PIXEL_FORMAT_COUNT
  std::string encoding;
  switch (pixel_format)
  {
    case 1:
      encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      break;

    case 2:
      encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      break;

    case 3:
      encoding = sensor_msgs::image_encodings::RGB8;
      break;

    case 6:
      encoding = sensor_msgs::image_encodings::RGB16;
      break;

    case 8:
      encoding = sensor_msgs::image_encodings::BGR8;
      break;

    case 9:
      encoding = sensor_msgs::image_encodings::BGR16;
      break;

    case 11:
      encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      break;

    case 13:
      encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;

    default:
      DBG_SIM_WRN("Unsupported pixel type format(%d) !! ", pixel_format);
      encoding = sensor_msgs::image_encodings::RGB8;
      break;
  }

  return encoding;
}

static void SetCameraInfoInManager(
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
    const cloisim::msgs::CameraSensor& msg,
    const std::string frame_id)
{
  if (infoManager == nullptr)
  {
    DBG_SIM_ERR("infoManager is null");
    return;
  }

  sensor_msgs::msg::CameraInfo camera_info_msg;

  const auto width = msg.image_size().x();
  const auto height = msg.image_size().y();

  // C parameters
  const auto cx_ = (width + 1.0) / 2.0;
  const auto cy_ = (height + 1.0) / 2.0;

  const auto hfov_ = msg.horizontal_fov();

  const auto computed_focal_length = width / (2.0 * tan(hfov_ / 2.0));

  // CameraInfo
  camera_info_msg.header.frame_id = frame_id;
  camera_info_msg.height = (uint32_t)height;
  camera_info_msg.width = (uint32_t)width;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  const auto hack_baseline = 0.0f;

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg.d[0] = msg.distortion().k1();
  camera_info_msg.d[1] = msg.distortion().k2();
  camera_info_msg.d[2] = msg.distortion().p1();
  camera_info_msg.d[3] = msg.distortion().p2();
  camera_info_msg.d[4] = msg.distortion().k3();

  // Original camera matrix
  camera_info_msg.k.fill(0.0);
  camera_info_msg.k[0] = computed_focal_length;
  camera_info_msg.k[4] = computed_focal_length;
  camera_info_msg.k[2] = cx_;
  camera_info_msg.k[5] = cy_;
  camera_info_msg.k[8] = 1.0;

  // rectification
  camera_info_msg.r.fill(0.0);
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[8] = 1.0;

  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.p.fill(0.0);
  camera_info_msg.p[0] = computed_focal_length;
  camera_info_msg.p[2] = cx_;
  camera_info_msg.p[3] = -computed_focal_length * hack_baseline;
  camera_info_msg.p[5] = computed_focal_length;
  camera_info_msg.p[6] = cy_;
  camera_info_msg.p[10] = 1.0;

  // set camera_info_manager
  infoManager->setCameraName(frame_id);
  infoManager->setCameraInfo(camera_info_msg);
}

static cloisim::msgs::CameraSensor GetCameraSensorMessage(
    cloisim_ros::zmq::Bridge* const bridge_ptr,
    const std::string camera_name = "")
{
  cloisim::msgs::Param request_msg;
  request_msg.set_name("request_camera_info");

  auto pVal = request_msg.mutable_value();
  pVal->set_type(cloisim::msgs::Any::STRING);
  pVal->set_string_value(camera_name);

  std::string serializedBuffer;
  request_msg.SerializeToString(&serializedBuffer);

  const auto reply = bridge_ptr->RequestReply(serializedBuffer);

  cloisim::msgs::CameraSensor cameraSensorInfo;
  if (reply.size() <= 0)
    DBG_SIM_ERR("Failed to get camera info, length(%ld)", reply.size());
  else
  {
    if (cameraSensorInfo.ParseFromString(reply) == false)
      DBG_SIM_ERR("Failed to Parsing Proto buffer buffer_ptr(%p) length(%ld)", reply.data(), reply.size());
  }

  return cameraSensorInfo;
}

#endif