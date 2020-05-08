/**
 *  @file   sim_bridge.cpp
 *  @date   2020-04-20
 *  @author Hyunseok Yang
 *  @brief
 *        ROS2 Sim Device class
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "sim_bridge/sim_bridge.hpp"
#include <cstring>
#include <thread>

using namespace std;

#define DEFAULT_MASTER_SIM_IP "127.0.0.1"
#define DEFAULT_PORT_MANAGER_PORT 25554

SimBridge::SimBridge()
  : simMasterAddress(DEFAULT_MASTER_SIM_IP)
  , simPortManagerPortNumber(DEFAULT_PORT_MANAGER_PORT)
  , m_pCtx(nullptr)
  , m_pPub(nullptr)
  , m_pSub(nullptr)
  , m_pReq(nullptr)
  , m_pRep(nullptr)
  , m_pSockTx(nullptr)
  , m_pSockRx(nullptr)
  , m_retryPortRequest(1700)
  , lastErrMsg("")
{
  m_pCtx = zmq_ctx_new();
}

SimBridge::~SimBridge()
{
  if (m_pCtx)
  {
    zmq_ctx_term(m_pCtx);
    m_pCtx = nullptr;
  }
}

/**
 * @brief setup simdevice
 *
 * @param mode : bit slection ex) Setup(Mode::PUB|Mode::SUB)
 */
bool SimBridge::Setup(const unsigned char mode)
{
  bool result = true;

  if (mode & Mode::SUB)
  {
    result &= SetupSubscriber();
  }

  if (mode & Mode::PUB)
  {
    result &= SetupPublisher();
  }

  if (mode & Mode::SERVICE)
  {
    result &= SetupService();
  }

  if (mode & Mode::CLIENT)
  {
    result &= SetupClient();
  }

  if (result == false)
  {
    DBG_SIM_ERR("Error::%s", lastErrMsg.c_str());
  }

  return result;
}

bool SimBridge::SetupCommon(void* const targetSocket)
{
  if (zmq_setsockopt(targetSocket, ZMQ_RECONNECT_IVL, &reconnect_ivl_min, sizeof(reconnect_ivl_min)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_setsockopt(targetSocket, ZMQ_LINGER, &lingerPeriod, sizeof(lingerPeriod)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool SimBridge::SetupSubscriber()
{
  m_pSub = zmq_socket(m_pCtx, ZMQ_SUB);

  if (m_pSub == nullptr)
  {
    lastErrMsg = "NULL Socket for sub!!";
    return false;
  }

  if (!SetupCommon(m_pSub))
  {
    return false;
  }

  if (zmq_setsockopt(m_pSub, ZMQ_CONFLATE, &keepOnlyLastMsg, sizeof(keepOnlyLastMsg)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_setsockopt(m_pSub, ZMQ_RCVTIMEO, &recv_timeout, sizeof(recv_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0)
  {
    lastErrMsg = "msg init failed:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  m_pSockRx = m_pSub;

  return true;
}

bool SimBridge::SetupPublisher()
{
  m_pPub = zmq_socket(m_pCtx, ZMQ_PUB);

  if (!SetupCommon(m_pPub))
  {
    return false;
  }

  m_pSockTx = m_pPub;

  return true;
}

bool SimBridge::SetupService()
{
  m_pRep = zmq_socket(m_pCtx, ZMQ_REP);

  if (!SetupCommon(m_pRep))
  {
    return false;
  }

  if (zmq_setsockopt(m_pRep, ZMQ_RCVTIMEO, &recv_timeout, sizeof(recv_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0)
  {
    lastErrMsg = "msg init failed:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  m_pSockTx = m_pRep;
  m_pSockRx = m_pRep;

  return true;
}

bool SimBridge::SetupClient()
{
  m_pReq = zmq_socket(m_pCtx, ZMQ_REQ);

  if (!SetupCommon(m_pReq))
  {
    return false;
  }

  if (zmq_setsockopt(m_pReq, ZMQ_RCVTIMEO, &recv_timeout, sizeof(recv_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0)
  {
    lastErrMsg = "msg init failed:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  m_pSockTx = m_pReq;
  m_pSockRx = m_pReq;

  return true;
}

bool SimBridge::Connect(const unsigned char mode, const string hashKey)
{
  bool result = true;

  uint16_t port = 0;
  do
  {
    this_thread::sleep_for(chrono::milliseconds(m_retryPortRequest));
    port = RequestBridgePortNumber(hashKey);
  } while (port == 0);

  // socket configuration
  result &= Setup(mode);

  // socket connect
  if (mode & Mode::SUB)
  {
    result &= ConnectSubscriber(port, hashKey);
  }

  if (mode & Mode::PUB)
  {
    result &= ConnectPublisher(port, hashKey);
  }

  if (mode & Mode::SERVICE)
  {
    result &= ConnectService(port, hashKey);
  }

  if (mode & Mode::CLIENT)
  {
    result &= ConnectClient(port, hashKey);
  }

  if (result == false)
  {
    DBG_SIM_ERR("Connect()::%s", lastErrMsg.c_str());
  }

  return result;
}

bool SimBridge::ConnectSubscriber(const uint16_t port, const string hashKey)
{
  size_t nHashTag = hash<string>{}(hashKey);
  if (zmq_setsockopt(m_pSub, ZMQ_SUBSCRIBE, &nHashTag, tagSize))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  const string bridgeAddress = GetSimBridgeAddress(port);
  DBG_SIM_MSG("Sub bridgeAddress=[%s]", bridgeAddress.c_str());

  if (zmq_connect(m_pSub, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectSubscriber Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool SimBridge::ConnectPublisher(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = hash<string>{}(hashKey);

  const string bridgeAddress = GetSimBridgeAddress(port);
  DBG_SIM_MSG("Pub bridge address=[%s]", bridgeAddress.c_str());

  if (zmq_connect(m_pPub, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectPublisher Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool SimBridge::ConnectService(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = hash<string>{}(hashKey);

  const string bridgeAddress = GetSimBridgeAddress(port);
  DBG_SIM_MSG("Service bridge address=[%s]", bridgeAddress.c_str());

  if (zmq_connect(m_pRep, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectService Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool SimBridge::ConnectClient(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = hash<string>{}(hashKey);

  const string bridgeAddress = GetSimBridgeAddress(port);
  DBG_SIM_MSG("Client for service bridge address=[%s]", bridgeAddress.c_str());
  if (zmq_connect(m_pReq, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectClient Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool SimBridge::Disconnect(const unsigned char mode)
{
  bool result = true;

  if ((mode == 0 || (mode & Mode::SUB)) && m_pSub)
  {
    zmq_msg_close(&m_msgRx);
    result &= CloseSocket(m_pSub);
  }

  if ((mode == 0 || (mode & Mode::PUB)) && m_pPub)
  {
    result &= CloseSocket(m_pPub);
  }

  if ((mode == 0 || (mode & Mode::SERVICE)) && m_pRep)
  {
    zmq_msg_close(&m_msgRx);
    result &= CloseSocket(m_pRep);
  }

  if ((mode == 0 || (mode & Mode::CLIENT)) && m_pReq)
  {
    result &= CloseSocket(m_pReq);
  }

  m_pSockTx = nullptr;
  m_pSockRx = nullptr;

  return result;
}

bool SimBridge::Reconnect(const unsigned char mode, const string hashKey)
{
  bool result = true;
  result &= Disconnect(mode);
  result &= Connect(mode, hashKey);
  return result;
}

bool SimBridge::CloseSocket(void*& target)
{
  if (target == nullptr)
  {
    return false;
  }

  zmq_close(target);
  target = nullptr;

  return true;
}

bool SimBridge::Receive(void** buffer, int& bufferLength, bool isNonBlockingMode)
{
  if (&m_msgRx == nullptr || m_pSockRx == nullptr)
  {
    return false;
  }

  bufferLength = zmq_msg_recv(&m_msgRx, m_pSockRx, (isNonBlockingMode)? ZMQ_DONTWAIT:0);

  if (bufferLength == 0)
  {
    return false;
  }

  *buffer = zmq_msg_data(&m_msgRx);

  if (*buffer == nullptr)
  {
    return false;
  }

  // Get only Contents without tag
  auto ptr = static_cast<unsigned char *>(*buffer);
  *buffer = (void* )(ptr + tagSize);
  bufferLength -= tagSize;

  return true;
}

bool SimBridge::Send(const void* buffer, const int bufferLength, bool isNonBlockingMode)
{
  zmq_msg_t msg;
  if (m_pSockTx == nullptr || zmq_msg_init_size(&msg, tagSize + bufferLength) < 0)
  {
    return false;
  }

  // Set hash Tag
  memcpy(zmq_msg_data(&msg), &m_nHashTagTx, tagSize);
  memcpy((void*)((uint8_t*)zmq_msg_data(&msg) + tagSize), buffer, bufferLength);

  /* Send the message to the socket */
  if (zmq_msg_send(&msg, m_pSockTx, (isNonBlockingMode)? ZMQ_DONTWAIT:0) < 0)
  {
    return false;
  }

	zmq_msg_close(&msg);

  return true;
}

uint16_t SimBridge::RequestBridgePortNumber(const string key)
{
  static const ushort sizeOfPortNumber = 7;
  static char portNum[sizeOfPortNumber];

  auto sockReq = zmq_socket(m_pCtx, ZMQ_REQ);
  const auto portManagerAddress = GetPortManagerAddress();

  DBG_SIM_INFO("bridge addressess: %s", portManagerAddress.c_str());

  if (zmq_connect(sockReq, portManagerAddress.c_str()))
  {
    DBG_SIM_ERR("Connect Err for publish: %s", zmq_strerror(zmq_errno()));
    return 0;
  }

  int ret;

  // Send request to server
  ret = zmq_send(sockReq, key.data(), key.size(), 0);
  DBG_SIM_INFO(" --> Request(%d): %s", ret, key.c_str());

  // receive port nubmer from port manager server
  memset(portNum, 0x0, sizeof(portNum));

  //  Wait for next request from client
  ret = zmq_recv(sockReq, portNum, sizeOfPortNumber, 0);
  DBG_SIM_INFO(" <-- Receive(%d): %s", ret, portNum);

  if ((sockReq == nullptr) || zmq_close(sockReq))
  {
    DBG_SIM_ERR("ZMQ socket for request closing failed");
  }

  return atoi(portNum);
}