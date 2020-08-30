/**
 *  @file   sim_bridge.hpp
 *  @date   2020-04-21
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

#ifndef _SIM_BRIDGE_H_
#define _SIM_BRIDGE_H_

#include "sim_bridge/debug_log.h"
#include <string>
#include <zmq.h>

class SimBridge
{
public:
  enum Mode
  {
    SUB     = 0b00001,
    PUB     = 0b00010,
    SERVICE = 0b00100,
    CLIENT  = 0b01000
  };

public:
  void SetSimBridgeAddress(const std::string ip_address)
  {
    simBridgeIP = ip_address;
  }

  void SetBridgeManagerPort(const uint16_t port)
  {
    simBridgeManagerPort = port;
  }

public:
  SimBridge();
  ~SimBridge();

  bool Connect(const unsigned char mode, const std::string hashKey = "");
  bool Disconnect(const unsigned char mode = 0);
  bool Reconnect(const unsigned char mode, const std::string hashKey = "");

  bool Receive(void** buffer, int& bufferLength, bool isNonBlockingMode = false);
  bool Send(const void* buffer, const int bufferLength, bool isNonBlockingMode = false);

  void SetRetryRequestPeriod(const int value)
  {
    retryPortRequest_ = value;
  }

private:
  const bool useTCP = true;
  const uint8_t tagSize = 8; // The size of zmq packet header tag

  const int keepOnlyLastMsg = 1;
  const int reconnect_ivl_min = 500;
  const int lingerPeriod = 0;
  const int recv_timeout = 2000; // milliseconds

  std::string simBridgeIP;
  uint16_t simBridgeManagerPort;

  void* pCtx_;

  void* pPub_;
  void* pSub_;
  void* pReq_;
  void* pRep_;

  zmq_msg_t m_msgRx; // for subscriber and reply
  size_t m_nHashTagTx; // for publisher and request

  void* pSockTx_; // for Send function
  void* pSockRx_; // for Recieve function

  int retryPortRequest_;

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
  std::string GetPortManagerAddress()
  {
    return GetSimBridgeAddress(simBridgeManagerPort);
  }

  std::string GetSimBridgeAddress(const uint16_t port)
  {
    return std::string((useTCP)? "tcp":"udp") + "://" + simBridgeIP + ":" + std::to_string(port);
  }

  uint16_t RequestBridgePortNumber(const std::string key);
};
#endif