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
  void SetSimMasterAddress(const std::string value)
  {
    simMasterAddress = value;
  }

  void SetPortManagerPort(const uint16_t value)
  {
    simPortManagerPortNumber = value;
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
    m_retryPortRequest = value;
  }

private:
  const bool useTCP = true;
  const uint8_t tagSize = 8; // The size of zmq packet header tag

  const int keepOnlyLastMsg = 1;
  const int reconnect_ivl_min = 500;
  const int lingerPeriod = 0;
  const int recv_timeout = 2000; // milliseconds

  std::string simMasterAddress;
  uint16_t simPortManagerPortNumber;

  void* m_pCtx;

  void* m_pPub;
  void* m_pSub;
  void* m_pReq;
  void* m_pRep;

  zmq_msg_t m_msgRx; // for subscriber and reply
  size_t m_nHashTagTx; // for publisher and request

  void* m_pSockTx; // for Send function
  void* m_pSockRx; // for Recieve function

  int m_retryPortRequest;

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
    return GetSimBridgeAddress(simPortManagerPortNumber);
  }

  std::string GetSimBridgeAddress(const uint16_t port)
  {
    return std::string((useTCP)? "tcp":"udp") + "://" + simMasterAddress + ":" + std::to_string(port);
  }

  uint16_t RequestBridgePortNumber(const std::string key);
};
#endif