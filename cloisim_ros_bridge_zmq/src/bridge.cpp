/**
 *  @file   cloisim_ros_bridge_zmq.cpp
 *  @date   2021-01-14
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

#include "cloisim_ros_bridge_zmq/bridge.hpp"
#include <cstring>
#include <thread>

using namespace std;
using namespace cloisim_ros::zmq;

#define DEFAULT_CLOISIM_BRIDGE_IP "127.0.0.1"

Bridge::Bridge()
  : pCtx_(nullptr)
  , pPub_(nullptr)
  , pSub_(nullptr)
  , pReq_(nullptr)
  , pRep_(nullptr)
  , pSockTx_(nullptr)
  , pSockRx_(nullptr)
  , lastErrMsg("")
{
  const auto env_bridge_ip = getenv("CLOISIM_BRIDGE_IP");

  if (env_bridge_ip == nullptr)
  {
    // DBG_SIM_WRN("env for CLOISIM_BRIDGE_IP is null, will use default.");
    SetBridgeAddress(DEFAULT_CLOISIM_BRIDGE_IP);
  }
  else
  {
    SetBridgeAddress(string(env_bridge_ip));
  }

  pCtx_ = zmq_ctx_new();

  // DBG_SIM_INFO("bridge_ip = %s", bridgeAddr_.c_str());
}


Bridge::~Bridge()
{
  if (pCtx_)
  {
    zmq_ctx_term(pCtx_);
    pCtx_ = nullptr;
  }
}

/**
 * @brief setup cloisim_ros
 *
 * @param mode : bit slection ex) Setup(Mode::PUB|Mode::SUB)
 */
bool Bridge::Setup(const unsigned char mode)
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

bool Bridge::SetupCommon(void* const targetSocket)
{
  if (zmq_setsockopt(targetSocket, ZMQ_CONNECT_TIMEOUT, &connect_timeout, sizeof(connect_timeout)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_setsockopt(targetSocket, ZMQ_RECONNECT_IVL, &reconnect_ivl_min, sizeof(reconnect_ivl_min)))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  if (zmq_setsockopt(targetSocket, ZMQ_RECONNECT_IVL_MAX, &reconnect_ivl_max, sizeof(reconnect_ivl_max)))
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

bool Bridge::SetupSubscriber()
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

  if (zmq_setsockopt(pSub_, ZMQ_CONFLATE, &keep_only_last_msg, sizeof(keep_only_last_msg)))
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

bool Bridge::SetupPublisher()
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

bool Bridge::SetupService()
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

bool Bridge::SetupClient()
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

bool Bridge::Connect(const unsigned char mode, const uint16_t port, const string hashKey)
{
  bool result = true;

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

bool Bridge::ConnectSubscriber(const uint16_t port, const string hashKey)
{
  size_t nHashTag = hash<string>{}(hashKey);
  if (zmq_setsockopt(pSub_, ZMQ_SUBSCRIBE, &nHashTag, tagSize))
  {
    lastErrMsg = "SetSock Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  const string bridgeAddress = GetAddress(port);
  DBG_SIM_MSG("address(%s)", bridgeAddress.c_str());

  if (zmq_connect(pSub_, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectSubscriber Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool Bridge::ConnectPublisher(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = hash<string>{}(hashKey);

  const string bridgeAddress = GetAddress(port);
  DBG_SIM_MSG("address(%s)", bridgeAddress.c_str());

  if (zmq_connect(pPub_, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectPublisher Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool Bridge::ConnectService(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = hash<string>{}(hashKey);

  const string bridgeAddress = GetAddress(port);
  DBG_SIM_MSG("address(%s)", bridgeAddress.c_str());

  if (zmq_connect(pRep_, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectService Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool Bridge::ConnectClient(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = hash<string>{}(hashKey);

  const string bridgeAddress = GetAddress(port);
  DBG_SIM_MSG("address(%s)", bridgeAddress.c_str());
  if (zmq_connect(pReq_, bridgeAddress.c_str()) < 0)
  {
    lastErrMsg = "ConnectClient Err:" + string(zmq_strerror(zmq_errno()));
    return false;
  }

  return true;
}

bool Bridge::Disconnect(const unsigned char mode)
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

bool Bridge::CloseSocket(void*& target)
{
  if (target == nullptr)
  {
    return false;
  }

  zmq_close(target);
  target = nullptr;

  return true;
}

bool Bridge::Receive(void** buffer, int& bufferLength, bool isNonBlockingMode)
{
  if (&m_msgRx == nullptr || pSockRx_ == nullptr)
  {
    DBG_SIM_ERR("Cannot Receive data due to uninitialized pointer m_msgRx(%p) or pSockRx_(%p)", (void*)&m_msgRx, pSockRx_);
    return false;
  }

  if ((bufferLength = zmq_msg_recv(&m_msgRx, pSockRx_, (isNonBlockingMode) ? ZMQ_DONTWAIT : 0)) < 0)
  {
    DBG_SIM_ERR("failed to receive ZMQ message: err(%s) length(%d)", zmq_strerror(zmq_errno()), bufferLength);
    return false;
  }

  if ((*buffer = zmq_msg_data(&m_msgRx)) == nullptr)
  {
    return false;
  }

  // Get only Contents without tag
  auto ptr = static_cast<unsigned char *>(*buffer);
  *buffer = (void* )(ptr + tagSize);
  bufferLength -= tagSize;

  return true;
}

bool Bridge::Send(const void* buffer, const int bufferLength, bool isNonBlockingMode)
{
  zmq_msg_t msg;
  if (pSockTx_ == nullptr || zmq_msg_init_size(&msg, tagSize + bufferLength) < 0)
  {
    DBG_SIM_ERR("Cannot Send data due to uninitialized pointer msg(%p) or pSockTx_(%p)", (void*)&msg, pSockTx_);
    return false;
  }

  // Set hash Tag
  memcpy(zmq_msg_data(&msg), &m_nHashTagTx, tagSize);
  memcpy((void*)((uint8_t*)zmq_msg_data(&msg) + tagSize), buffer, bufferLength);

  /* Send the message to the socket */
  if (zmq_msg_send(&msg, pSockTx_, (isNonBlockingMode) ? ZMQ_DONTWAIT : 0) < 0)
  {
    return false;
  }

	zmq_msg_close(&msg);

  return true;
}

std::string Bridge::RequestReply(std::string request_data)
{
  string reply_data;

  if (request_data.size() > 0)
  {
    Send(request_data.data(), request_data.size());

    void *buffer_ptr = nullptr;
    int bufferLength = 0;
    const auto succeeded = Receive(&buffer_ptr, bufferLength);

    if (succeeded)
    {
      reply_data.assign(static_cast<char *>(buffer_ptr), bufferLength);
    }
    else
    {
      DBG_SIM_ERR("Faild to get reply buffer, length(%d)", bufferLength);
    }
  }

  return reply_data;
}
