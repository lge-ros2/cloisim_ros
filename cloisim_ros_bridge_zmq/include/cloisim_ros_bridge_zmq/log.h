/**
 *  @file   log.h
 *  @date   2020-04-21
 *  @author Hyunseok Yang
 *  @brief
 *      the following are UBUNTU/LINUX, and MacOS ONLY terminal color codes.
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */
#ifndef CLOISIM_ROS_BRIDGE_ZMQ__LOG_H_
#define CLOISIM_ROS_BRIDGE_ZMQ__LOG_H_

#include "cloisim_ros_bridge_zmq/term_color.h"

#pragma GCC system_header

#define __SIM_LOG(COLOR_CODE, STR_FORMAT, ...) \
  printf(COLOR_CODE "[%s] " STR_FORMAT RESET "\n", __FUNCTION__, ## __VA_ARGS__)
//   printf(COLOR_CODE "[%s][%d] " STR_FORMAT RESET "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define DBG_SIM_INFO(STR_FORMAT, ...) __SIM_LOG(CYAN, STR_FORMAT, ## __VA_ARGS__)
#define DBG_SIM_WRN(STR_FORMAT, ...) __SIM_LOG(YELLOW, STR_FORMAT, ## __VA_ARGS__)
#define DBG_SIM_MSG(STR_FORMAT, ...) __SIM_LOG(GREEN, STR_FORMAT, ## __VA_ARGS__)
#define DBG_SIM_ERR(STR_FORMAT, ...) __SIM_LOG(BOLDRED, STR_FORMAT, ## __VA_ARGS__)

#define __SIM_LOG_WITHOUT_FUNC(COLOR_CODE, STR_FORMAT, ...) \
  printf(COLOR_CODE STR_FORMAT RESET "\n", ## __VA_ARGS__)
//   printf(COLOR_CODE "[%s][%d] " STR_FORMAT RESET "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define DBG_INFO(STR_FORMAT, ...) __SIM_LOG_WITHOUT_FUNC(CYAN, STR_FORMAT, ## __VA_ARGS__)
#define DBG_WRN(STR_FORMAT, ...) __SIM_LOG_WITHOUT_FUNC(YELLOW, STR_FORMAT, ## __VA_ARGS__)
#define DBG_MSG(STR_FORMAT, ...) __SIM_LOG_WITHOUT_FUNC(GREEN, STR_FORMAT, ## __VA_ARGS__)
#define DBG_ERR(STR_FORMAT, ...) __SIM_LOG_WITHOUT_FUNC(BOLDRED, STR_FORMAT, ## __VA_ARGS__)

#endif  // CLOISIM_ROS_BRIDGE_ZMQ__LOG_H_
