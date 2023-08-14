/**
 *  @file   cloisim_ros_bridge_zmq.hpp
 *  @date   2021-01-14
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Sim bridge base class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef _CLOISIM_ROS_BRIDGE_ZMQ_HPP_
#define _CLOISIM_ROS_BRIDGE_ZMQ_HPP_

#include "log.h"
#include <string>
#include <zmq.h>

namespace cloisim_ros
{
namespace zmq
{
class Bridge
{
 public:
  enum Mode
  {
    SUB = 0b00001,
    PUB = 0b00010,
    SERVICE = 0b00100,
    CLIENT = 0b01000
  };

 public:
  void SetBridgeAddress(const std::string ip_address)
  {
    bridgeAddr_ = ip_address;
  }

 public:
  Bridge();
  ~Bridge();

  bool Connect(const unsigned char mode, const uint16_t port, const std::string hashKey = "");
  bool Disconnect(const unsigned char mode = 0);

  bool Receive(void** buffer, int& bufferLength, bool isNonBlockingMode = false);
  bool Send(const void* buffer, const int bufferLength, bool isNonBlockingMode = false);

  std::string RequestReply(std::string request_data);

 private:
  const bool useTCP = true;
  const uint8_t tagSize = 8;  // The size of zmq packet header tag

  const int keep_only_last_msg = 1;
  const int connect_timeout = 0;
  const int reconnect_ivl_min = 1000;
  const int reconnect_ivl_max = 5000;
  const int lingerPeriod = 0;
  const int recv_timeout = 3000;  // milliseconds

  std::string bridgeAddr_;

  void* pCtx_;

  void* pPub_;
  void* pSub_;
  void* pReq_;
  void* pRep_;

  zmq_msg_t m_msgRx;         // for subscriber and reply
  std::size_t m_nHashTagTx;  // for publisher and request

  void* pSockTx_;            // for Send function
  void* pSockRx_;            // for Recieve function

  std::string lastErrMsg;

 private:
  bool Setup(const unsigned char mode);
  bool SetupCommon(void* const targetSocket);
  bool SetupSubscriber();
  bool SetupPublisher();
  bool SetupService();
  bool SetupClient();

  // TODO : need to implement action/client model later
  // bool SetupAction() { return false; };
  // bool SetupClient() { return false; };

  bool ConnectSubscriber(const uint16_t port, const std::string hashKey);
  bool ConnectPublisher(const uint16_t port, const std::string hashKey);
  bool ConnectService(const uint16_t port, const std::string hashKey);
  bool ConnectClient(const uint16_t port, const std::string hashKey);

  // TODO : need to implement action/client model later
  // bool ConnectAction() { return false; };
  // bool ConnectClient() { return false; };

  bool CloseSocket(void*& target);

 private:
  std::string GetAddress(const uint16_t port)
  {
    return std::string((useTCP) ? "tcp" : "udp") + "://" + bridgeAddr_ + ":" + std::to_string(port);
  }

  std::size_t GetHashCode(const std::string value)
  {
    return std::hash<std::string>{}(value);
  }
};
}  // namespace zmq
}  // namespace cloisim_ros
#endif