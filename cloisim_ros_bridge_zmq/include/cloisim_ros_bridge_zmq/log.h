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

#include <cxxabi.h>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <cstring>
#include <string>
#include <typeinfo>

#include "term_color.h"

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

inline std::string Demangle(const char * name)
{
  int status = 0;
  auto demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
  std::string result = (status == 0 && demangled) ? demangled : name;
  std::free(demangled);
  return result;
}

inline std::string GetDemangledName(const void *)
{
  return "";
}

template < typename T >
inline std::string GetDemangledName(const T *obj_ptr)
{
  if (!obj_ptr) {return "";}
  return Demangle(typeid(*obj_ptr).name());
}

template < typename T >
inline std::string GetDemangledName(const std::shared_ptr < T > &obj_ptr)
{
  return GetDemangledName(obj_ptr.get());
}

enum class LogLevel { Info, Warn, Error };

inline const char * LogLevelStr(LogLevel lv)
{
  switch (lv) {
    case LogLevel::Info:  return "I";
    case LogLevel::Warn:  return "W";
    case LogLevel::Error: return "E";
  }
  return "?";
}

inline const char * LogColor(LogLevel lv)
{
  switch (lv) {
    case LogLevel::Info:  return CYAN;
    case LogLevel::Warn:  return YELLOW;
    case LogLevel::Error: return BOLDRED;
  }
  return RESET;
}

inline std::string NowTimestamp()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto ms = duration_cast < milliseconds > (now.time_since_epoch()) % 1000;
  const auto t = system_clock::to_time_t(now);

  std::tm tm {};
  localtime_r(&t, &tm);

  std::ostringstream oss;
  // MMDD HH:MM:SS.mmm
  oss << std::setfill('0')
      << std::setw(2) << (tm.tm_mon + 1)
      << std::setw(2) << tm.tm_mday
      << " "
      << std::put_time(&tm, "%H:%M:%S")
      << "." << std::setw(3) << ms.count();
  return oss.str();
}

inline void LogImpl(
  LogLevel level,
  const char * file,
  int line,
  const char * func,
  const std::string & type_name,
  const void * obj_addr,
  const std::string & message)
{
  const char * color = LogColor(level);
  const char * reset = RESET;
  const auto ts = NowTimestamp();

  std::cerr
      << color
      << LogLevelStr(level) << ts << " "
      << file << ":" << line << "] "
      << ((func && std::strcmp(func, "operator()") != 0) ? (std::string(func) + "()] ") : "")
      // << type_name << "@"
      // << obj_addr << " | "
      << message
      << reset
      << std::endl;
}

inline const void * ToRawPtr(std::nullptr_t) {return nullptr;}

template < typename T >
inline const T * ToRawPtr(const T * p) {return p;}

template < typename T >
inline const T * ToRawPtr(const std::shared_ptr < T > &sp) {return sp.get();}

#define LOG_IMPL_(LEVEL, OBJ, STREAM_EXPR) \
  do { \
    const auto * obj_ptr = ToRawPtr(OBJ); \
    std::ostringstream _log_oss; \
    _log_oss << STREAM_EXPR; \
    LogImpl((LEVEL), __FILE_NAME__, __LINE__, __func__, \
      GetDemangledName(obj_ptr), \
      static_cast < const void * > (obj_ptr), \
      _log_oss.str()); \
  } while (0)

#define LOG_I(OBJ, STREAM_EXPR) LOG_IMPL_(LogLevel::Info, OBJ, STREAM_EXPR)
#define LOG_W(OBJ, STREAM_EXPR) LOG_IMPL_(LogLevel::Warn, OBJ, STREAM_EXPR)
#define LOG_E(OBJ, STREAM_EXPR) LOG_IMPL_(LogLevel::Error, OBJ, STREAM_EXPR)

#endif  // CLOISIM_ROS_BRIDGE_ZMQ__LOG_H_
