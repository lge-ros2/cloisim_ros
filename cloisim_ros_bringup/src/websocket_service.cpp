/**
 *  @file   websocket_service.cpp
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

#include <cloisim_ros_bringup/websocket_service.hpp>

using namespace cloisim_ros;
using namespace std;

WebSocketService::WebSocketService()
{
  const auto env_service_port = getenv("CLOISIM_SERVICE_PORT");
  const auto service_port = string((env_service_port == nullptr) ? "8080" : env_service_port);

  new (this) WebSocketService(service_port);
}

WebSocketService::WebSocketService(const string service_port)
{
  const auto env_bridge_ip = getenv("CLOISIM_BRIDGE_IP");
  const auto bridge_ip = string((env_bridge_ip == nullptr) ? "127.0.0.1" : env_bridge_ip);

  new (this) WebSocketService(bridge_ip, service_port);
}

WebSocketService::WebSocketService(const string bridge_ip, const string service_port)
{
  uri = "ws://" + bridge_ip + ":" + service_port;

  // Set logging to be pretty verbose (everything except message payloads)
  c.set_access_channels(websocketpp::log::alevel::connect);
  c.clear_access_channels(websocketpp::log::alevel::frame_header);
  c.clear_access_channels(websocketpp::log::alevel::control);
  c.clear_access_channels(websocketpp::log::alevel::endpoint);
  c.clear_access_channels(websocketpp::log::alevel::disconnect);
  c.clear_error_channels(websocketpp::log::elevel::all);
  c.set_error_channels(websocketpp::log::elevel::fatal);

  // Initialize ASIO
  c.init_asio();

  // Register our message handler
  c.set_open_handler(bind(&WebSocketService::on_open, this, placeholders::_1));
  c.set_message_handler(bind(&WebSocketService::on_message, this, placeholders::_1, placeholders::_2));
}

void WebSocketService::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
  // cout << "DUMP: " << msg->get_payload() << endl;
  const auto payload = msg->get_payload();

  Json::Value root;
  reader.parse(payload, root, false);

  result = root["result"];

  c.close(hdl, websocketpp::close::status::going_away, "");
  // cout << "close" << endl;
}

void WebSocketService::on_open(websocketpp::connection_hdl hdl)
{
  const string request_msg = "{'command':'device_list', 'filter':'" + target_filter + "'}";

  // std::cout << "Open: " << std::endl;
  c.send(hdl, request_msg, websocketpp::frame::opcode::value::TEXT);
}

Json::Value WebSocketService::Run()
{
  try
  {
    websocketpp::lib::error_code ec;
    client::connection_ptr con = c.get_connection(uri + "/control", ec);
    if (ec)
    {
      cout << "could not create connection because: " << ec.message() << endl;
    }
    else
    {
      // Note that connect here only requests a connection. No network messages are
      // exchanged until the event loop starts running in the next line.
      c.connect(con);

      // Start the ASIO io_service run loop
      // this will cause a single connection to be made to the server. c.run()
      // will exit when this connection is closed.
      c.run();

      c.reset();
    }
  }
  catch (websocketpp::exception const &e)
  {
    cout << "Exception:" << e.what() << endl;
  }

  // cout << result << endl << result.size() << endl;

  return result;
}