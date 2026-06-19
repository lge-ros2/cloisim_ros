/**
 *  @file   base_generate_tf_test.cpp
 *  @date   2026-05-22
 *  @brief  Regression tests for Base::GenerateTF.
 *  @copyright
 *      SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <cloisim_msgs/param.pb.h>
#include <cloisim_msgs/pose.pb.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cloisim_ros_base/base.hpp>
#include <cloisim_ros_base/param_helper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mock_bridge_server.hpp"

using namespace std::literals::chrono_literals;

namespace cloisim_ros
{
namespace
{

class TestBaseNode : public Base
{
public:
  TestBaseNode()
  : Base("cloisim_ros_base_generate_tf_test")
  {
    Start(true);
  }

  ~TestBaseNode() override
  {
    Stop();
  }

  zmq::Bridge * CreateInfoBridge(uint16_t port, const std::string & hash_key)
  {
    auto * bridge = CreateBridge();
    if (bridge != nullptr) {
      bridge->Connect(zmq::Bridge::Mode::CLIENT, port, hash_key);
    }
    return bridge;
  }

  void RequestStaticTransforms(zmq::Bridge * bridge)
  {
    SetStaticTransforms(bridge);
  }

protected:
  void Initialize() override {}
  void Deinitialize() override {}
};

class BaseGenerateTFTests : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    // Base::Start registers an on_shutdown callback capturing `this`.
    // Let the process teardown reclaim ROS state instead of triggering
    // shutdown callbacks after the fixture-owned nodes are gone.
  }

  void SetUp() override
  {
    publisher_node_ = std::make_shared<TestBaseNode>();
    listener_node_ = std::make_shared<rclcpp::Node>("tf_listener");

    tf_subscription_ = listener_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf",
      rclcpp::QoS(100),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        received_messages_.push_back(*msg);
      });

    static_tf_subscription_ = listener_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf_static",
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        received_static_messages_.push_back(*msg);
      });

    executor_.add_node(publisher_node_);
    executor_.add_node(listener_node_);

    // Give DDS discovery a brief window before publishing the first TF.
    SpinFor(200ms);
  }

  void TearDown() override
  {
    executor_.remove_node(listener_node_);
    executor_.remove_node(publisher_node_);

    tf_subscription_.reset();
    static_tf_subscription_.reset();
    listener_node_.reset();
    publisher_node_.reset();

    std::lock_guard<std::mutex> lock(received_mutex_);
    received_messages_.clear();
    received_static_messages_.clear();
  }

  static std::string BuildPoseMessage(
    const std::string & header_key,
    const std::string & parent_frame_id,
    const std::string & child_frame_id)
  {
    cloisim::msgs::Pose pose;
    pose.set_name(child_frame_id);
    pose.mutable_header()->mutable_stamp()->set_sec(123);
    pose.mutable_header()->mutable_stamp()->set_nsec(456);

    auto * header_data = pose.mutable_header()->add_data();
    header_data->set_key(header_key);
    header_data->add_value(parent_frame_id);

    pose.mutable_position()->set_x(1.25);
    pose.mutable_position()->set_y(-2.5);
    pose.mutable_position()->set_z(3.75);
    pose.mutable_orientation()->set_x(0.0);
    pose.mutable_orientation()->set_y(0.0);
    pose.mutable_orientation()->set_z(0.0);
    pose.mutable_orientation()->set_w(1.0);

    std::string serialized;
    pose.SerializeToString(&serialized);
    return serialized;
  }

  static cloisim::msgs::Param BuildStaticTransformReply(
    const std::string & parent_frame_id,
    const std::string & child_frame_id)
  {
    cloisim::msgs::Param reply;

    cloisim::msgs::Any root_value;
    root_value.set_type(cloisim::msgs::Any_ValueType_NONE);
    param::Set(reply, "static_transforms", root_value);

    auto * static_tf_link = reply.add_children();
    cloisim::msgs::Any parent_value;
    parent_value.set_type(cloisim::msgs::Any_ValueType_STRING);
    parent_value.set_string_value(parent_frame_id);
    param::Set(*static_tf_link, "parent_frame_id", parent_value);

    auto * pose_param = static_tf_link->add_children();
    cloisim::msgs::Any pose_value;
    pose_value.set_type(cloisim::msgs::Any_ValueType_POSE3D);
    auto * pose = pose_value.mutable_pose3d_value();
    pose->set_name(child_frame_id);
    pose->mutable_position()->set_x(0.1);
    pose->mutable_position()->set_y(-0.2);
    pose->mutable_position()->set_z(0.3);
    pose->mutable_orientation()->set_x(0.0);
    pose->mutable_orientation()->set_y(0.0);
    pose->mutable_orientation()->set_z(0.0);
    pose->mutable_orientation()->set_w(1.0);
    param::Set(*pose_param, "pose", pose_value);

    return reply;
  }

  void SpinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  bool WaitForTransform(
    const std::string & expected_parent_frame,
    const std::string & expected_child_frame,
    geometry_msgs::msg::TransformStamped * matched_transform = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;

    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();

      {
        std::lock_guard<std::mutex> lock(received_mutex_);
        for (const auto & tf_message : received_messages_) {
          for (const auto & transform : tf_message.transforms) {
            if (transform.header.frame_id == expected_parent_frame &&
              transform.child_frame_id == expected_child_frame)
            {
              if (matched_transform != nullptr) {
                *matched_transform = transform;
              }
              return true;
            }
          }
        }
      }

      std::this_thread::sleep_for(10ms);
    }

    return false;
  }

  bool WaitForStaticTransform(
    const std::string & expected_parent_frame,
    const std::string & expected_child_frame,
    geometry_msgs::msg::TransformStamped * matched_transform = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;

    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();

      {
        std::lock_guard<std::mutex> lock(received_mutex_);
        for (const auto & tf_message : received_static_messages_) {
          for (const auto & transform : tf_message.transforms) {
            if (transform.header.frame_id == expected_parent_frame &&
              transform.child_frame_id == expected_child_frame)
            {
              if (matched_transform != nullptr) {
                *matched_transform = transform;
              }
              return true;
            }
          }
        }
      }

      std::this_thread::sleep_for(10ms);
    }

    return false;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<TestBaseNode> publisher_node_;
  rclcpp::Node::SharedPtr listener_node_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_subscription_;
  std::mutex received_mutex_;
  std::vector<tf2_msgs::msg::TFMessage> received_messages_;
  std::vector<tf2_msgs::msg::TFMessage> received_static_messages_;
};

TEST_F(BaseGenerateTFTests, PublishesTransformWhenParentFrameIdHeaderIsPresent)
{
  auto serialized = BuildPoseMessage("parent_frame_id", "base_link", "laser_link");

  publisher_node_->GenerateTF(serialized.data(), static_cast<int>(serialized.size()));

  geometry_msgs::msg::TransformStamped transform;
  ASSERT_TRUE(WaitForTransform("base_link", "laser_link", &transform));
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.25);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, -2.5);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 3.75);
}

TEST_F(BaseGenerateTFTests, FallsBackToLegacyFrameIdHeader)
{
  auto serialized = BuildPoseMessage("frame_id", "legacy_parent", "camera_link");

  publisher_node_->GenerateTF(serialized.data(), static_cast<int>(serialized.size()));

  geometry_msgs::msg::TransformStamped transform;
  ASSERT_TRUE(WaitForTransform("legacy_parent", "camera_link", &transform));
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.25);
}

TEST_F(BaseGenerateTFTests, RetriesStaticTransformRequestUntilSimulatorResponds)
{
  auto request_count = 0;
  auto info_server = std::make_unique<cloisim_ros::test::MockBridgeServer>();
  const auto info_port = info_server->StartInfoServer(
    [&request_count](const std::string & request_name) -> cloisim::msgs::Param {
      if (request_name != "request_static_transforms") {
        return cloisim::msgs::Param();
      }

      ++request_count;
      if (request_count < 3) {
        return cloisim::msgs::Param();
      }

      return BuildStaticTransformReply("base_link", "body_base_link");
    });

  ASSERT_GT(info_port, 0);

  auto * bridge = publisher_node_->CreateInfoBridge(info_port, "static-transform-test");
  ASSERT_NE(bridge, nullptr);

  publisher_node_->RequestStaticTransforms(bridge);

  geometry_msgs::msg::TransformStamped transform;
  ASSERT_TRUE(WaitForStaticTransform("base_link", "body_base_link", &transform));
  EXPECT_EQ(request_count, 3);
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 0.1);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, -0.2);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.3);
}

}  // namespace
}  // namespace cloisim_ros

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
