/**
 *  @file   helper.h
 *  @date   2020-05-22
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 driver Sim helper
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _DRIVER_SIM_HELPER_H_
#define _DRIVER_SIM_HELPER_H_
#include <sensor_msgs/image_encodings.hpp>

static std::string GetImageEncondingType(const uint32_t pixel_format)
{
  // 	UNKNOWN_PIXEL_FORMAT = 0, L_INT8, L_INT16,
  // 	RGB_INT8 = 3, RGBA_INT8, BGRA_INT8, RGB_INT16, RGB_INT32,
  // 	BGR_INT8 = 8, BGR_INT16, BGR_INT32,
  // 	R_FLOAT16 = 11, RGB_FLOAT16 = 12, R_FLOAT32 = 13, RGB_FLOAT32,
  // 	BAYER_RGGB8, BAYER_RGGR8, BAYER_GBRG8, BAYER_GRBG8,
  // 	PIXEL_FORMAT_COUNT
  std::string encoding;
  switch (pixel_format)
  {
    case 1:
      encoding = sensor_msgs::image_encodings::MONO8;
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

    case 13:
      encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;

    default:
      DBG_SIM_WRN("Unsupported pixel type format !!");
      encoding = sensor_msgs::image_encodings::RGB8;
      break;
  }

  return encoding;
}

#endif