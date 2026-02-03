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

#define DEFAULT_CLOISIM_BRIDGE_IP "127.0.0.1"

using string = std::string;

namespace cloisim_ros::zmq
{
Bridge::Bridge()
: pCtx_(nullptr)
  , pPub_(nullptr)
  , pSub_(nullptr)
  , pReq_(nullptr)
  , pRep_(nullptr)
  , pSockTx_(nullptr)
  , pSockRx_(nullptr)
  , lastErrMsg_("")
{
  const auto env_bridge_ip = getenv("CLOISIM_BRIDGE_IP");

  // if (env_bridge_ip == nullptr)
  //   LOG_W(this, "env for CLOISIM_BRIDGE_IP is null, will use default.");

  SetBridgeAddress((env_bridge_ip == nullptr) ? DEFAULT_CLOISIM_BRIDGE_IP : string(env_bridge_ip));

  pCtx_ = zmq_ctx_new();

  // LOG_I(this, "bridge_ip=" << bridgeAddr_);
}

Bridge::~Bridge()
{
  if (pCtx_) {
    if (!ctx_shutdown_called_.exchange(true)) {
      zmq_ctx_shutdown(pCtx_);
    }

    if (!ctx_term_called_.exchange(true)) {
      zmq_ctx_term(pCtx_);
    }

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

  if (mode & Mode::SUB) {result &= SetupSubscriber();}

  if (mode & Mode::PUB) {result &= SetupPublisher();}

  if (mode & Mode::SERVICE) {result &= SetupService();}

  if (mode & Mode::CLIENT) {result &= SetupClient();}

  if (result == false) {LOG_E(this, "Error::" << lastErrMsg_);}

  return result;
}

bool Bridge::SetupCommon(void * const socket)
{
  if (zmq_setsockopt(socket, ZMQ_CONNECT_TIMEOUT, &connect_timeout, sizeof(connect_timeout))) {
    lastErrMsg_ = "SetSock Err:" + GetLastErrorMessage();
    return false;
  }

  if (zmq_setsockopt(socket, ZMQ_RECONNECT_IVL, &reconnect_ivl_min_ms,
      sizeof(reconnect_ivl_min_ms)))
  {
    lastErrMsg_ = "SetSock Err:" + GetLastErrorMessage();
    return false;
  }

  if (zmq_setsockopt(
      socket, ZMQ_RECONNECT_IVL_MAX, &reconnect_ivl_max_ms, sizeof(reconnect_ivl_max_ms)))
  {
    lastErrMsg_ = "SetSock Err:" + GetLastErrorMessage();
    return false;
  }

  if (zmq_setsockopt(socket, ZMQ_LINGER, &lingerPeriod, sizeof(lingerPeriod))) {
    lastErrMsg_ = "SetSock Err:" + GetLastErrorMessage();
    return false;
  }

  return true;
}

bool Bridge::SetupSubscriber()
{
  // condition check
  if (pSub_ != nullptr) {
    lastErrMsg_ = "pSub_ is Already setup!!!";
    return false;
  } else if (pRep_ != nullptr || pReq_ != nullptr) {
    lastErrMsg_ = "pReq_ or pRep_ is Already setup!!!";
    return false;
  }

  pSub_ = zmq_socket(pCtx_, ZMQ_SUB);

  if (pSub_ == nullptr) {
    lastErrMsg_ = "NULL Socket for sub!!";
    return false;
  }

  if (!SetupCommon(pSub_)) {return false;}

  if (zmq_setsockopt(pSub_, ZMQ_CONFLATE, &keep_only_last_msg, sizeof(keep_only_last_msg))) {
    lastErrMsg_ = "SetSock Err:" + GetLastErrorMessage();
    return false;
  }

  if (zmq_setsockopt(pSub_, ZMQ_RCVTIMEO, &recv_timeout_ms, sizeof(recv_timeout_ms))) {
    lastErrMsg_ = "SetSock Err:" + GetLastErrorMessage();
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0) {
    lastErrMsg_ = "msg init failed:" + GetLastErrorMessage();
    return false;
  }

  pSockRx_ = pSub_;

  return true;
}

bool Bridge::SetupPublisher()
{
  // condition check
  if (pPub_ != nullptr) {
    lastErrMsg_ = "pPub_ is Already setup!!!";
    return false;
  } else if (pRep_ != nullptr || pReq_ != nullptr) {
    lastErrMsg_ = "pReq_ or pRep_ is Already setup!!!";
    return false;
  }

  pPub_ = zmq_socket(pCtx_, ZMQ_PUB);

  if (!SetupCommon(pPub_)) {return false;}

  pSockTx_ = pPub_;

  return true;
}

bool Bridge::SetupService()
{
  // condition check
  if (pRep_ != nullptr) {
    lastErrMsg_ = "pRep_ is Already setup!!!";
    return false;
  } else if (pSub_ != nullptr || pPub_ != nullptr) {
    lastErrMsg_ = "pSub_ or pPub_ is Already setup!!!";
    return false;
  }

  pRep_ = zmq_socket(pCtx_, ZMQ_REP);

  if (!SetupCommon(pRep_)) {return false;}

  if (zmq_msg_init(&m_msgRx) < 0) {
    lastErrMsg_ = "msg init failed, err=" + GetLastErrorMessage();
    return false;
  }

  pSockTx_ = pRep_;
  pSockRx_ = pRep_;

  return true;
}

bool Bridge::SetupClient()
{
  // condition check
  if (pReq_ != nullptr) {
    lastErrMsg_ = "pRep_ is Already setup!!!";
    return false;
  } else if (pSub_ != nullptr || pPub_ != nullptr) {
    lastErrMsg_ = "pSub_ or pPub_ is Already setup!!!";
    return false;
  }

  pReq_ = zmq_socket(pCtx_, ZMQ_REQ);

  if (!SetupCommon(pReq_)) {return false;}

  if (zmq_setsockopt(pReq_, ZMQ_RCVTIMEO, &recv_timeout_ms, sizeof(recv_timeout_ms))) {
    lastErrMsg_ = "SetSock err=" + GetLastErrorMessage();
    return false;
  }

  if (zmq_msg_init(&m_msgRx) < 0) {
    lastErrMsg_ = "msg init failed, err=" + GetLastErrorMessage();
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

  if (result) {
    // socket connect
    if (mode & Mode::SUB) {result &= ConnectSubscriber(port, hashKey);}

    if (mode & Mode::PUB) {result &= ConnectPublisher(port, hashKey);}

    if (mode & Mode::SERVICE) {result &= ConnectService(port, hashKey);}

    if (mode & Mode::CLIENT) {result &= ConnectClient(port, hashKey);}

    if (!result) {LOG_E(this, "Connect()::" << lastErrMsg_);}
  }

  return result;
}

bool Bridge::ConnectSubscriber(const uint16_t port, const string hashKey)
{
  const auto nHashTag = GetHashCode(hashKey);
  if (zmq_setsockopt(pSub_, ZMQ_SUBSCRIBE, &nHashTag, tagSize)) {
    lastErrMsg_ = "SetSock err=" + GetLastErrorMessage();
    return false;
  }

  const auto bridgeAddress = GetAddress(port);
  LOG_I(this, "addr=" << bridgeAddress << " hash=" << std::hex << nHashTag << std::dec);

  if (zmq_connect(pSub_, bridgeAddress.c_str()) < 0) {
    lastErrMsg_ = "ConnectSubscriber err=" + GetLastErrorMessage();
    return false;
  }

  return true;
}

bool Bridge::ConnectPublisher(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = GetHashCode(hashKey);

  const auto bridgeAddress = GetAddress(port);
  LOG_I(this, "addr=" << bridgeAddress << " hash=" << std::hex << m_nHashTagTx << std::dec);

  if (zmq_connect(pPub_, bridgeAddress.c_str()) < 0) {
    lastErrMsg_ = "ConnectPublisher err=" + GetLastErrorMessage();
    return false;
  }

  return true;
}

bool Bridge::ConnectService(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = GetHashCode(hashKey);

  const auto bridgeAddress = GetAddress(port);
  LOG_I(this, "addr=" << bridgeAddress << " hash=" << std::hex << m_nHashTagTx << std::dec);

  if (zmq_connect(pRep_, bridgeAddress.c_str()) < 0) {
    lastErrMsg_ = "ConnectService err=" + GetLastErrorMessage();
    return false;
  }

  return true;
}

bool Bridge::ConnectClient(const uint16_t port, const string hashKey)
{
  m_nHashTagTx = GetHashCode(hashKey);

  const auto bridgeAddress = GetAddress(port);
  LOG_I(this, "addr=" << bridgeAddress << " hash=" << std::hex << m_nHashTagTx << std::dec);

  if (zmq_connect(pReq_, bridgeAddress.c_str()) < 0) {
    lastErrMsg_ = "ConnectClient err=" + GetLastErrorMessage();
    return false;
  }

  return true;
}

bool Bridge::Disconnect(const unsigned char mode)
{
  // LOG_W(this, "Bridge disconnect");

  if (pCtx_ && !ctx_shutdown_called_.exchange(true)) {
    zmq_ctx_shutdown(pCtx_);
  }

  auto result = true;

  if ((mode == 0 || (mode & Mode::SUB)) && pSub_) {
    zmq_msg_close(&m_msgRx);
    result &= CloseSocket(pSub_);
  }

  if ((mode == 0 || (mode & Mode::SERVICE)) && pRep_) {
    zmq_msg_close(&m_msgRx);
    result &= CloseSocket(pRep_);
  }

  if ((mode == 0 || (mode & Mode::PUB)) && pPub_) {result &= CloseSocket(pPub_);}

  if ((mode == 0 || (mode & Mode::CLIENT)) && pReq_) {result &= CloseSocket(pReq_);}

  pSockTx_ = nullptr;
  pSockRx_ = nullptr;

  return result;
}

bool Bridge::CloseSocket(void * & target)
{
  if (target == nullptr) {
    LOG_E(this, "null target");
    return false;
  }

  zmq_close(target);
  target = nullptr;

  return true;
}

bool Bridge::Receive(void ** buffer, int & bufferLength, bool is_non_blocking_mode)
{
  if (pSockRx_ == nullptr) {
    LOG_E(this, "Cannot Receive data due to uninitialized pointer pSockRx=" << pSockRx_);
    return false;
  }

  if ((bufferLength = zmq_msg_recv(&m_msgRx, pSockRx_,
    (is_non_blocking_mode) ? ZMQ_DONTWAIT : 0)) < 0)
  {
    // LOG_E(this,
    //     "Failed to receive message len=" << bufferLength << " err=" << GetLastErrorMessage();
    return false;
  }

  if ((*buffer = zmq_msg_data(&m_msgRx)) == nullptr) {return false;}

  // Get only Contents without tag
  auto ptr = static_cast<unsigned char *>(*buffer);
  *buffer = reinterpret_cast<void *>(ptr + tagSize);
  bufferLength -= tagSize;

  return true;
}

bool Bridge::Send(const void * buffer, const int bufferLength, bool is_non_blocking_mode)
{
  zmq_msg_t msg;
  if (pSockTx_ == nullptr || zmq_msg_init_size(&msg, tagSize + bufferLength) < 0) {
    LOG_E(this,
        "Cannot Send data due to uninitialized pointer, msg=" << (void *)&msg << " or pSockTx=" <<
        pSockTx_);
    return false;
  }

  // Set hash Tag
  memcpy(zmq_msg_data(&msg), &m_nHashTagTx, tagSize);
  memcpy(
    reinterpret_cast<void *>(reinterpret_cast<uint8_t *>(zmq_msg_data(&msg)) + tagSize), buffer,
    bufferLength);

  /* Send the message to the socket */
  if (zmq_msg_send(&msg, pSockTx_, (is_non_blocking_mode) ? ZMQ_DONTWAIT : 0) < 0) {
    return false;
  }

  zmq_msg_close(&msg);

  return true;
}

std::string Bridge::RequestReply(const std::string & request_data)
{
  string reply_data;

  if (request_data.size() > 0) {
    Send(request_data.data(), request_data.size());

    void * buffer_ptr = nullptr;
    int bufferLength = 0;
    const auto succeeded = Receive(&buffer_ptr, bufferLength);

    if (succeeded) {
      reply_data.assign(static_cast<char *>(buffer_ptr), bufferLength);
    } else {
      LOG_E(this, "Failed to get reply buffer, length=" << bufferLength);
    }
  }

  return reply_data;
}

}  // namespace cloisim_ros::zmq
