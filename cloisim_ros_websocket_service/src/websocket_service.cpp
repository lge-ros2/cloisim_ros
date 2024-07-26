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

#include "cloisim_ros_websocket_service/websocket_service.hpp"

using namespace std::placeholders;
using namespace std::literals::chrono_literals;
using string = std::string;
using std::cout;
using std::endl;

namespace cloisim_ros
{
namespace
{
constexpr auto DEFAULT_WS_SERVICE_PORT = "8080";
constexpr auto DEFAULT_WS_SERVICE_BRIDGE_ADDRESS = "127.0.0.1";
}  // namespace

WebSocketService::WebSocketService()
{
  const auto env_service_port = getenv("CLOISIM_SERVICE_PORT");
  const auto service_port =
    string((env_service_port == nullptr) ? DEFAULT_WS_SERVICE_PORT : env_service_port);

  new (this) WebSocketService(service_port);
}

WebSocketService::WebSocketService(const string service_port)
{
  const auto env_bridge_ip = getenv("CLOISIM_BRIDGE_IP");
  const auto bridge_ip =
    string((env_bridge_ip == nullptr) ? DEFAULT_WS_SERVICE_BRIDGE_ADDRESS : env_bridge_ip);

  new (this) WebSocketService(bridge_ip, service_port);
}

WebSocketService::WebSocketService(const string bridge_ip, const string service_port)
: base_uri_("ws://" + bridge_ip + ":" + service_port)
  , thread_(nullptr)
  , target_filter("")
  , payload_("")
  , is_reply_received_(false)
  , is_connected_(false)
{
  // Set logging to be pretty verbose (everything except message payloads)
  client_.clear_access_channels(websocketpp::log::alevel::all);
  client_.clear_error_channels(websocketpp::log::elevel::all);
  client_.set_error_channels(websocketpp::log::elevel::rerror);

  // client_.set_access_channels(websocketpp::log::alevel::connect);
  // client_.set_access_channels(websocketpp::log::alevel::app);
  // client_.set_access_channels(websocketpp::log::alevel::http);
  // client_.set_access_channels(websocketpp::log::alevel::access_core);
  // client_.set_access_channels(websocketpp::log::alevel::disconnect);
  // client_.set_access_channels(websocketpp::log::alevel::fail);
  // client_.set_error_channels(websocketpp::log::elevel::all);
  // client_.set_error_channels(websocketpp::log::elevel::info);
  // client_.set_error_channels(websocketpp::log::elevel::fatal);

  // Register our message handler
  client_.set_open_handler(bind(&WebSocketService::on_open, this, _1));
  client_.set_close_handler(bind(&WebSocketService::on_close, this, _1));
  client_.set_message_handler(bind(&WebSocketService::on_message, this, _1, _2));

  client_.set_reuse_addr(true);

  // Initialize ASIO
  client_.init_asio();
  client_.start_perpetual();

  thread_.reset(new websocketpp::lib::thread(&client::run, &client_));
}

WebSocketService::~WebSocketService()
{
  // cout << __FUNCTION__ << endl;
  client_.stop_perpetual();
  client_.stop();

  if (thread_->joinable()) {thread_->join();}
  thread_ = nullptr;
}

void WebSocketService::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
  (void)hdl;
  // cout << __FUNCTION__ << ":: DUMP=" << msg->get_payload() << endl;
  payload_ = msg->get_payload();
  is_reply_received_ = true;
}

void WebSocketService::on_open(websocketpp::connection_hdl hdl)
{
  (void)hdl;
  // cout << __FUNCTION__ << endl;
  conn_hdl_ = hdl;
  is_connected_ = true;
}

void WebSocketService::on_close(websocketpp::connection_hdl hdl)
{
  (void)hdl;
  cout << "Disconnected from CLOiSim" << endl;
  is_connected_ = false;
}

void WebSocketService::Request()
{
  if (IsConnected() && is_reply_received_ == false) {
    try {
      const auto request_msg = "{'command':'device_list', 'filter':'" + target_filter + "'}";
      client_.send(conn_hdl_, request_msg, websocketpp::frame::opcode::value::TEXT);
      // cout << "Request" << endl;
      std::this_thread::sleep_for(300ms);
    } catch (const websocketpp::exception & e) {
      cout << IsConnected() << ", " << is_reply_received_ << endl;
      cout << "Exception in Request() because: " << e.what() << endl;
      std::this_thread::sleep_for(1000ms);
    }
  }
}

std::string WebSocketService::PopPayload()
{
  // cout << "(" << payload_.size() << ") " << payload_ << endl;
  const auto payload_to_return = payload_;
  payload_.clear();
  is_reply_received_ = false;
  return payload_to_return;
}

void WebSocketService::Run()
{
  if (IsConnected() == false) {
    const auto target_uri = base_uri_ + "/control";
    // cout << __FUNCTION__ << ", " << target_uri << endl;

    try {
      websocketpp::lib::error_code ec;
      const auto conn = client_.get_connection(target_uri, ec);

      if (!ec) {
        // Note that connect here only requests a connection. No network messages are
        // exchanged until the event loop starts running in the next line.
        client_.connect(conn);
      } else {
        cout << "could not create connection because: " << ec.message()
             << " target_uri: " << target_uri << endl;
      }
    } catch (websocketpp::exception const & e) {
      cout << "Exception:" << e.what() << endl;
    } catch (websocketpp::lib::error_code e) {
      cout << "Error Code: " << e.message() << endl;
    } catch (...) {
      cout << "other exception" << endl;
    }

    std::this_thread::sleep_for(500ms);
  }
}
}  // namespace cloisim_ros
