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

#include <sim_bridge/sim_bridge.hpp>
#include <cstring>
#include <thread>

using namespace std;

#define DEFAULT_SIM_BRIDGE_IP "127.0.0.1"
#define DEFAULT_SIM_BRIDGE_MANAGER_PORT 25554


SimBridge::SimBridge()
  : simBridgeIP(DEFAULT_SIM_BRIDGE_IP)
  , simBridgeManagerPort(DEFAULT_SIM_BRIDGE_MANAGER_PORT)
  , pCtx_(nullptr)
  , pPub_(nullptr)
  , pSub_(nullptr)
  , pReq_(nullptr)
  , pRep_(nullptr)
  , pSockTx_(nullptr)
  , pSockRx_(nullptr)
  , retryPortRequest_(1700)
  , lastErrMsg("")
{
  auto env_sim_bridge_ip = getenv("SIM_BRIDGE_IP");
  auto env_sim_bridge_manager_port = getenv("SIM_BRIDGE_MANAGER_PORT");

  if (env_sim_bridge_ip == nullptr)
  {
    DBG_SIM_WRN("[SIM_BRIDGE] env for SIM_BRIDGE_IP is null, will use default.");
  }
  else
  {
    SetSimBridgeAddress(string(env_sim_bridge_ip));
  }

  if (env_sim_bridge_manager_port == nullptr)
  {
    DBG_SIM_WRN("[SIM_BRIDGE] env for SIM_BRIDGE_MANAGER_PORT is null, will use default.");
  }
  else
  {
    const auto port = atoi(env_sim_bridge_manager_port);
    SetBridgeManagerPort(port);
  }

  pCtx_ = zmq_ctx_new();

  DBG_SIM_INFO("[SIM_BRIDGE] bridge_ip = %s, bridge_manager = %d", simBridgeIP.c_str(), simBridgeManagerPort);
}


SimBridge::~SimBridge()
{
  if (pCtx_)
  {
    zmq_ctx_term(pCtx_);
    pCtx_ = nullptr;
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
  // condition check
  if (pSub_ != nullptr)
  {
    lastErrMsg = "pSub_ is Already setup!!!";
    return false;
  }
  else if (pRep_ != nullptr || pReq_ != nullptr)
  {
    lastErrMsg = "pReq_ or pRep_ is Already setup!!!";
    return false;
  }

  pSub_ = zmq_socket(pCtx_, ZMQ_SUB);

  if (pSub_ == nullptr)
  {
    lastErrMsg = "NULL Socket for sub!!";
    return false;
  }

  if (!SetupCommon(pSub_))
  {
    return false;
  }

  if (zmq_setsockopt(pSub_, ZMQ_CONFLATE, &keepOnlyLastMsg, sizeof(keepOnlyLastMsg)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_setsockopt(pSub_, ZMQ_RCVTIMEO, &recv_timeout, sizeof(recv_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0)
  {
    lastErrMsg = "msg init failed:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  pSockRx_ = pSub_;

  return true;
}

bool SimBridge::SetupPublisher()
{
  // condition check
  if (pPub_ != nullptr)
  {
    lastErrMsg = "pPub_ is Already setup!!!";
    return false;
  }
  else if (pRep_ != nullptr || pReq_ != nullptr)
  {
    lastErrMsg = "pReq_ or pRep_ is Already setup!!!";
    return false;
  }

  pPub_ = zmq_socket(pCtx_, ZMQ_PUB);

  if (!SetupCommon(pPub_))
  {
    return false;
  }

  pSockTx_ = pPub_;

  return true;
}

bool SimBridge::SetupService()
{
  // condition check
  if (pRep_ != nullptr)
  {
    lastErrMsg = "pRep_ is Already setup!!!";
    return false;
  }
  else if (pSub_ != nullptr || pPub_ != nullptr)
  {
    lastErrMsg = "pSub_ or pPub_ is Already setup!!!";
    return false;
  }

  pRep_ = zmq_socket(pCtx_, ZMQ_REP);

  if (!SetupCommon(pRep_))
  {
    return false;
  }

  if (zmq_setsockopt(pRep_, ZMQ_RCVTIMEO, &recv_timeout, sizeof(recv_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0)
  {
    lastErrMsg = "msg init failed:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  pSockTx_ = pRep_;
  pSockRx_ = pRep_;

  return true;
}

bool SimBridge::SetupClient()
{
  // condition check
  if (pReq_ != nullptr)
  {
    lastErrMsg = "pRep_ is Already setup!!!";
    return false;
  }
  else if (pSub_ != nullptr || pPub_ != nullptr)
  {
    lastErrMsg = "pSub_ or pPub_ is Already setup!!!";
    return false;
  }

  pReq_ = zmq_socket(pCtx_, ZMQ_REQ);

  if (!SetupCommon(pReq_))
  {
    return false;
  }

  if (zmq_setsockopt(pReq_, ZMQ_RCVTIMEO, &recv_timeout, sizeof(recv_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0)
  {
    lastErrMsg = "msg init failed:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  pSockTx_ = pReq_;
  pSockRx_ = pReq_;

  return true;
}

bool SimBridge::Connect(const unsigned char mode, const string hashKey)
{
  bool result = true;

  uint16_t port = 0;
  do
  {
    this_thread::sleep_for(chrono::milliseconds(retryPortRequest_));
    port = RequestBridgePortNumber(hashKey);
  } while (port == 0);

  // socket configuration
  result &= Setup(mode);

  if (result)
  {
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

    if (!result)
    {
      DBG_SIM_ERR("Connect()::%s", lastErrMsg.c_str());
    }
  }

  return result;
}

bool SimBridge::ConnectSubscriber(const uint16_t port, const string hashKey)
{
  size_t nHashTag = hash<string>{}(hashKey);
  if (zmq_setsockopt(pSub_, ZMQ_SUBSCRIBE, &nHashTag, tagSize))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  const string bridgeAddress = GetSimBridgeAddress(port);
  DBG_SIM_MSG("Sub bridgeAddress=[%s]", bridgeAddress.c_str());

  if (zmq_connect(pSub_, bridgeAddress.c_str()) < 0)
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

  if (zmq_connect(pPub_, bridgeAddress.c_str()) < 0)
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

  if (zmq_connect(pRep_, bridgeAddress.c_str()) < 0)
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
  if (zmq_connect(pReq_, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectClient Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool SimBridge::Disconnect(const unsigned char mode)
{
  bool result = true;

  if ((mode == 0 || (mode & Mode::SUB)) && pSub_)
  {
    zmq_msg_close(&m_msgRx);
    result &= CloseSocket(pSub_);
  }

  if ((mode == 0 || (mode & Mode::PUB)) && pPub_)
  {
    result &= CloseSocket(pPub_);
  }

  if ((mode == 0 || (mode & Mode::SERVICE)) && pRep_)
  {
    zmq_msg_close(&m_msgRx);
    result &= CloseSocket(pRep_);
  }

  if ((mode == 0 || (mode & Mode::CLIENT)) && pReq_)
  {
    result &= CloseSocket(pReq_);
  }

  pSockTx_ = nullptr;
  pSockRx_ = nullptr;

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
  if (&m_msgRx == nullptr || pSockRx_ == nullptr)
  {
    DBG_SIM_ERR("Cannot Receive data due to uninitialized pointer m_msgRx(%p) or pSockRx_(%p)", &m_msgRx, pSockRx_);
    return false;
  }

  bufferLength = zmq_msg_recv(&m_msgRx, pSockRx_, (isNonBlockingMode)? ZMQ_DONTWAIT:0);

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
  if (pSockTx_ == nullptr || zmq_msg_init_size(&msg, tagSize + bufferLength) < 0)
  {
    DBG_SIM_ERR("Cannot Send data due to uninitialized pointer msg(%p) or pSockTx_(%p)", &msg, pSockTx_);
    return false;
  }

  // Set hash Tag
  memcpy(zmq_msg_data(&msg), &m_nHashTagTx, tagSize);
  memcpy((void*)((uint8_t*)zmq_msg_data(&msg) + tagSize), buffer, bufferLength);

  /* Send the message to the socket */
  if (zmq_msg_send(&msg, pSockTx_, (isNonBlockingMode)? ZMQ_DONTWAIT:0) < 0)
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

  auto sockReq = zmq_socket(pCtx_, ZMQ_REQ);
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