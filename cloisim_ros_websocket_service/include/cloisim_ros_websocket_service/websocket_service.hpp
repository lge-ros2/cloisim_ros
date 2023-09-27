/**
 *  @file   websocket_service.hpp
 *  @date   2021-01-21
 *  @author Hyunseok Yang
 *  @brief
 *        websocket service for cloisim
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_WEBSOCKET_SERVICE__WEBSOCKET_SERVICE_HPP_
#define CLOISIM_ROS_WEBSOCKET_SERVICE__WEBSOCKET_SERVICE_HPP_

#define ASIO_STANDALONE

#include <string>

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

namespace cloisim_ros
{
class WebSocketService
{
 public:
  WebSocketService();
  explicit WebSocketService(const std::string service_port);
  explicit WebSocketService(const std::string bridge_ip, const std::string service_port);
  ~WebSocketService();

 private:
  std::string base_uri_;
  client client_;  // Create a client endpoint
  websocketpp::connection_hdl conn_hdl_;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread_;

  std::string target_filter;
  std::string payload_;

  bool is_reply_received_;
  bool is_connected_;

 private:
  void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg);
  void on_open(websocketpp::connection_hdl hdl);
  void on_close(websocketpp::connection_hdl hdl);

 public:
  void Request();

  bool IsConnected() const { return is_connected_; }

  std::string PopPayload();

  void SetTarget(const std::string target) { target_filter = target; }

  void Run();
};
}  // namespace cloisim_ros

#endif  // CLOISIM_ROS_WEBSOCKET_SERVICE__WEBSOCKET_SERVICE_HPP_
