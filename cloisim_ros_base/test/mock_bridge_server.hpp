/**
 *  @file   mock_bridge_server.hpp
 *  @date   2026-04-24
 *  @brief
 *        Mock ZMQ bridge server for testing CLOiSim ROS nodes.
 *        Emulates the simulator's ZMQ endpoints (REP for INFO, PUB for DATA).
 *  @copyright
 *      SPDX-License-Identifier: MIT
 */

#ifndef CLOISIM_ROS_BASE__TEST__MOCK_BRIDGE_SERVER_HPP_
#define CLOISIM_ROS_BASE__TEST__MOCK_BRIDGE_SERVER_HPP_

#include <zmq.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

#include <cloisim_msgs/any.pb.h>
#include <cloisim_msgs/camerasensor.pb.h>
#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/pose.pb.h>
#include <cloisim_ros_base/param_helper.hpp>

namespace cloisim_ros
{
namespace test
{

/// Compute hash tag matching the bridge's GetHashCode()
static inline std::size_t ComputeHashTag(const std::string & key)
{
  return std::hash<std::string>{}(key);
}

/// Extract port number from a ZMQ endpoint string like "tcp://127.0.0.1:12345"
static inline uint16_t ExtractPort(const std::string & endpoint)
{
  auto pos = endpoint.rfind(':');
  if (pos == std::string::npos) {return 0;}
  return static_cast<uint16_t>(std::stoi(endpoint.substr(pos + 1)));
}

/**
 * @brief Build a Param reply for "request_ros2" INFO requests.
 *
 * Returns a Param message containing topic_name and frame_id.
 */
static inline cloisim::msgs::Param BuildRos2ParamReply(
  const std::string & topic_name, const std::string & frame_id)
{
  cloisim::msgs::Param reply;

  // Set the top-level key "ros2"
  cloisim::msgs::Any ros2_val;
  ros2_val.set_type(cloisim::msgs::Any_ValueType_NONE);
  param::Set(reply, "ros2", ros2_val);

  // Add topic_name as child
  {
    auto * child = reply.add_children();
    cloisim::msgs::Any val;
    val.set_type(cloisim::msgs::Any_ValueType_STRING);
    val.set_string_value(topic_name);
    param::Set(*child, "topic_name", val);
  }

  // Add frame_id as child
  {
    auto * child = reply.add_children();
    cloisim::msgs::Any val;
    val.set_type(cloisim::msgs::Any_ValueType_STRING);
    val.set_string_value(frame_id);
    param::Set(*child, "frame_id", val);
  }

  return reply;
}

/**
 * @brief Build a Param reply for "request_transform" INFO requests.
 *
 * Returns a Param message containing an identity transform.
 */
static inline cloisim::msgs::Param BuildTransformReply(
  const std::string & frame_name = "test_frame")
{
  cloisim::msgs::Param reply;

  cloisim::msgs::Any transform_val;
  transform_val.set_type(cloisim::msgs::Any_ValueType_POSE3D);

  auto * pose = transform_val.mutable_pose3d_value();
  pose->set_name(frame_name);
  pose->mutable_position()->set_x(0.0);
  pose->mutable_position()->set_y(0.0);
  pose->mutable_position()->set_z(0.0);
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);

  param::Set(reply, "transform", transform_val);

  return reply;
}

/**
 * @brief Build a Param reply for "request_output_type" (used by Lidar).
 */
static inline cloisim::msgs::Param BuildOutputTypeReply(
  const std::string & output_type = "LaserScan")
{
  cloisim::msgs::Param reply;

  cloisim::msgs::Any val;
  val.set_type(cloisim::msgs::Any_ValueType_STRING);
  val.set_string_value(output_type);
  param::Set(reply, "output_type", val);

  return reply;
}

/**
 * @brief A mock ZMQ bridge server for testing CLOiSim ROS nodes.
 *
 * Provides REP (for INFO/config) and PUB (for sensor data) servers
 * that emulate the CLOiSim simulator's ZMQ endpoints.
 */
class MockBridgeServer
{
public:
  MockBridgeServer()
  : ctx_(zmq_ctx_new()), running_(false),
    rep_socket_(nullptr), pub_socket_(nullptr),
    info_port_(0), data_port_(0)
  {
  }

  ~MockBridgeServer()
  {
    Stop();
    if (ctx_) {
      zmq_ctx_term(ctx_);
      ctx_ = nullptr;
    }
  }

  // Non-copyable
  MockBridgeServer(const MockBridgeServer &) = delete;
  MockBridgeServer & operator=(const MockBridgeServer &) = delete;

  /**
   * @brief Start REP server for INFO requests.
   *
   * The handler receives a deserialized Param request and returns
   * a serialized Param response.
   *
   * @param handler Function that processes request name and returns Param reply
   * @return The port number the server is bound to
   */
  uint16_t StartInfoServer(
    std::function<cloisim::msgs::Param(const std::string &)> handler)
  {
    // Wrap into raw handler: Param → serialized Param
    auto raw_handler = [handler](const std::string & request_name) -> std::string {
        auto reply_param = handler(request_name);
        std::string serialized;
        reply_param.SerializeToString(&serialized);
        return serialized;
      };
    return StartInfoServerRaw(raw_handler);
  }

  /**
   * @brief Start REP server with raw serialized reply handler.
   *
   * The handler receives the request name string and returns
   * a raw serialized byte string (any protobuf message type).
   * This is needed for handlers that return non-Param messages
   * (e.g., CameraSensor for request_camera_info).
   *
   * @param handler Function that returns raw serialized reply bytes
   * @return The port number the server is bound to
   */
  uint16_t StartInfoServerRaw(
    std::function<std::string(const std::string &)> handler)
  {
    rep_socket_ = zmq_socket(ctx_, ZMQ_REP);
    if (!rep_socket_) {return 0;}

    // Set short receive timeout for responsive shutdown
    int timeout_ms = 100;
    zmq_setsockopt(rep_socket_, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    int linger = 0;
    zmq_setsockopt(rep_socket_, ZMQ_LINGER, &linger, sizeof(linger));

    if (zmq_bind(rep_socket_, "tcp://127.0.0.1:*") != 0) {return 0;}

    char endpoint[256];
    std::size_t endpoint_len = sizeof(endpoint);
    zmq_getsockopt(rep_socket_, ZMQ_LAST_ENDPOINT, endpoint, &endpoint_len);
    info_port_ = ExtractPort(std::string(endpoint));

    running_ = true;

    rep_thread_ = std::thread([this, handler]() {
          const std::size_t tag_size = 8;
          while (running_) {
            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int rc = zmq_msg_recv(&msg, rep_socket_, 0);
            if (rc < 0) {
              zmq_msg_close(&msg);
              continue; // timeout or error, retry
            }

        // Parse request: strip 8-byte hash tag, then parse protobuf Param
            auto msg_size = static_cast<std::size_t>(rc);
            std::string request_name;
            if (msg_size > tag_size) {
              auto * data = static_cast<char *>(zmq_msg_data(&msg));
              std::string raw(data + tag_size, msg_size - tag_size);

              cloisim::msgs::Param request;
              if (request.ParseFromString(raw)) {
                request_name = param::GetName(request);
              }
            }
            zmq_msg_close(&msg);

        // Get raw serialized response from handler
            std::string serialized = handler(request_name);

        // Send reply with 8-byte padding (tag stripped by client)
            std::string tagged_reply(tag_size, '\0');
            tagged_reply += serialized;
            zmq_send(rep_socket_, tagged_reply.data(), tagged_reply.size(), 0);
          }
    });

    return info_port_;
  }

  /**
   * @brief Start PUB server for DATA streaming.
   *
   * @return The port number the server is bound to
   */
  uint16_t StartDataServer()
  {
    pub_socket_ = zmq_socket(ctx_, ZMQ_PUB);
    if (!pub_socket_) {return 0;}

    int linger = 0;
    zmq_setsockopt(pub_socket_, ZMQ_LINGER, &linger, sizeof(linger));

    if (zmq_bind(pub_socket_, "tcp://127.0.0.1:*") != 0) {return 0;}

    char endpoint[256];
    std::size_t endpoint_len = sizeof(endpoint);
    zmq_getsockopt(pub_socket_, ZMQ_LAST_ENDPOINT, endpoint, &endpoint_len);
    data_port_ = ExtractPort(std::string(endpoint));

    return data_port_;
  }

  /**
   * @brief Publish sensor data on the PUB socket.
   *
   * @param hash_key The hash key for topic filtering (e.g., "robot_namepart_nameData")
   * @param serialized_data The protobuf-serialized sensor data
   */
  void PublishData(const std::string & hash_key, const std::string & serialized_data)
  {
    if (!pub_socket_) {return;}

    const std::size_t tag_size = 8;
    auto hash = ComputeHashTag(hash_key);

    std::size_t total_size = tag_size + serialized_data.size();
    zmq_msg_t msg;
    zmq_msg_init_size(&msg, total_size);

    auto * buf = static_cast<uint8_t *>(zmq_msg_data(&msg));
    std::memcpy(buf, &hash, tag_size);
    std::memcpy(buf + tag_size, serialized_data.data(), serialized_data.size());

    zmq_msg_send(&msg, pub_socket_, 0);
    zmq_msg_close(&msg);
  }

  void Stop()
  {
    running_ = false;

    if (rep_thread_.joinable()) {
      rep_thread_.join();
    }

    if (rep_socket_) {
      zmq_close(rep_socket_);
      rep_socket_ = nullptr;
    }
    if (pub_socket_) {
      zmq_close(pub_socket_);
      pub_socket_ = nullptr;
    }
  }

  uint16_t GetInfoPort() const {return info_port_;}
  uint16_t GetDataPort() const {return data_port_;}

private:
  void * ctx_;
  std::atomic<bool> running_;

  void * rep_socket_;
  void * pub_socket_;

  uint16_t info_port_;
  uint16_t data_port_;

  std::thread rep_thread_;
};

/**
 * @brief Default INFO request handler.
 *
 * Handles the common INFO requests made by most CLOiSim ROS nodes:
 * - "request_ros2" → returns topic_name and frame_id
 * - "request_transform" → returns identity transform
 * - "request_output_type" → returns "LaserScan" (for Lidar)
 * - "request_static_transforms" → returns empty Param
 *
 * @param topic_name The topic name to return
 * @param frame_id The frame_id to return
 * @param output_type The output_type to return (for Lidar nodes)
 */
static inline std::function<cloisim::msgs::Param(const std::string & )>
MakeDefaultInfoHandler(
  const std::string & topic_name = "test_topic",
  const std::string & frame_id = "test_frame",
  const std::string & output_type = "LaserScan")
{
  return [topic_name, frame_id, output_type](
    const std::string & request_name) -> cloisim::msgs::Param
         {
           if (request_name == "request_ros2") {
             return BuildRos2ParamReply(topic_name, frame_id);
           } else if (request_name == "request_transform") {
             return BuildTransformReply(frame_id);
           } else if (request_name == "request_output_type") {
             return BuildOutputTypeReply(output_type);
           } else if (request_name == "request_static_transforms") {
             return cloisim::msgs::Param();
           }
           return cloisim::msgs::Param();
         };
}

/**
 * @brief Helper to build NodeOptions with mock server bridge ports.
 */
static inline rclcpp::NodeOptions MakeNodeOptions(
  uint16_t info_port, uint16_t data_port,
  const std::string & robot_name = "")
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("bridge.Info", info_port);
  options.append_parameter_override("bridge.Data", data_port);
  if (!robot_name.empty()) {
    options.append_parameter_override("single_mode", true);
    options.append_parameter_override("single_mode.robotname", robot_name);
  }
  return options;
}

/**
 * @brief Raw INFO handler for Camera nodes.
 *
 * Handles request_camera_info by returning a CameraSensor protobuf
 * (not wrapped in Param), matching the simulator's actual behavior.
 * Other requests use the standard Param-based handler.
 */
static inline std::function<std::string(const std::string & )>
MakeCameraInfoHandler(
  const std::string & topic_name = "camera",
  const std::string & frame_id = "camera_link",
  uint32_t width = 320, uint32_t height = 240, double hfov = 1.047)
{
  auto param_handler = MakeDefaultInfoHandler(topic_name, frame_id);

  return [param_handler, width, height, hfov](
    const std::string & request_name) -> std::string
         {
           if (request_name == "request_camera_info") {
             cloisim::msgs::CameraSensor cam_info;
             cam_info.set_horizontal_fov(hfov);
             cam_info.mutable_image_size()->set_x(width);
             cam_info.mutable_image_size()->set_y(height);
             cam_info.mutable_distortion()->set_k1(0.0);
             cam_info.mutable_distortion()->set_k2(0.0);
             cam_info.mutable_distortion()->set_k3(0.0);
             cam_info.mutable_distortion()->set_p1(0.0);
             cam_info.mutable_distortion()->set_p2(0.0);

             std::string serialized;
             cam_info.SerializeToString(&serialized);
             return serialized;
           }

    // All other requests: use standard Param handler
           auto reply = param_handler(request_name);
           std::string serialized;
           reply.SerializeToString(&serialized);
           return serialized;
         };
}

}  // namespace test
}  // namespace cloisim_ros

#endif  // CLOISIM_ROS_BASE__TEST__MOCK_BRIDGE_SERVER_HPP_
