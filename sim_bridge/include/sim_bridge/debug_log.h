/**
 *  @file   term_color.h
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
#ifndef _DEBUG_LOG_H_
#define _DEBUG_LOG_H_

#include "term_color.h"

#pragma GCC system_header

#define _DBG_SIM(COLOR_CODE, STR_FORMAT, ...) \
    printf(COLOR_CODE "[%s][%d] " STR_FORMAT RESET "\n", __FUNCTION__, __LINE__, ## __VA_ARGS__)

#define DBG_SIM_INFO(STR_FORMAT, ...) _DBG_SIM(CYAN, STR_FORMAT, ## __VA_ARGS__)
#define DBG_SIM_WRN(STR_FORMAT, ...)  _DBG_SIM(YELLOW, STR_FORMAT, ## __VA_ARGS__)
#define DBG_SIM_MSG(STR_FORMAT, ...)  _DBG_SIM(GREEN, STR_FORMAT, ## __VA_ARGS__)
#define DBG_SIM_ERR(STR_FORMAT, ...)  _DBG_SIM(BOLDRED, STR_FORMAT, ## __VA_ARGS__)

#endif