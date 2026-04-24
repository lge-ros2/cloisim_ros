/**
 *  @file   test_micom_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Micom node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/micom.pb.h>
#include <cloisim_msgs/time.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "cloisim_ros_micom/micom.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class MicomNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();
    tx_server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("", "base_link"));
    tx_port_ = tx_server_->StartDataServer();  // PUB for Tx (data to node)

    ASSERT_GT(info_port_, 0);
    ASSERT_GT(tx_port_, 0);
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    node_.reset();
    server_->Stop();
    tx_server_->Stop();
    server_.reset();
    tx_server_.reset();
  }

  void CreateNode(const std::string & node_name = "test_micom")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("bridge.Info", info_port_);
    options.append_parameter_override("bridge.Tx", tx_port_);
    options.append_parameter_override("bridge.Rx", uint16_t(0));
    options.append_parameter_override("bridge.Tf", uint16_t(0));
    node_ = std::make_shared<cloisim_ros::Micom>(options, node_name);
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::unique_ptr<cloisim_ros::test::MockBridgeServer> tx_server_;
  std::shared_ptr<cloisim_ros::Micom> node_;
  uint16_t info_port_{0};
  uint16_t tx_port_{0};
};

TEST_F(MicomNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(MicomNodeTest, PublishesExpectedTopics)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();

  // Check for required topics
  std::vector<std::string> expected_topics = {"odom", "battery_state", "bumper", "ir", "uss"};

  for (const auto & expected : expected_topics) {
    bool found = false;
    for (const auto & [name, types] : topic_map) {
      if (name.find(expected) != std::string::npos) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Expected topic containing '" << expected << "' to be advertised";
  }
}

TEST_F(MicomNodeTest, SubscribesToCmdVel)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("cmd_vel") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'cmd_vel' subscription";
}

TEST_F(MicomNodeTest, HasResetOdometryService)
{
  CreateNode();

  auto service_map = node_->get_service_names_and_types();
  bool found = false;
  for (const auto & [name, types] : service_map) {
    if (name.find("reset_odometry") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'reset_odometry' service";
}

TEST_F(MicomNodeTest, ReceivesAndPublishesOdometry)
{
  CreateNode();

  nav_msgs::msg::Odometry received_odom;
  bool odom_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_micom_subscriber");
  auto sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SystemDefaultsQoS(),
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      received_odom = *msg;
      odom_received = true;
    });

  // Build a sample Micom protobuf message with odometry data
  cloisim::msgs::Micom micom;
  auto * time = micom.mutable_time();
  time->set_sec(100);
  time->set_nsec(0);

  auto * odom = micom.mutable_odom();
  // Odometry.pose is Vector3d (x, y, z=yaw)
  odom->mutable_pose()->set_x(1.0);
  odom->mutable_pose()->set_y(2.0);
  odom->mutable_pose()->set_z(0.5);  // yaw
  // Odometry.twist is Twist (linear + angular Vector3d)
  odom->mutable_twist()->mutable_linear()->set_x(0.5);
  odom->mutable_twist()->mutable_linear()->set_y(0.0);
  odom->mutable_twist()->mutable_linear()->set_z(0.0);
  odom->mutable_twist()->mutable_angular()->set_x(0.0);
  odom->mutable_twist()->mutable_angular()->set_y(0.0);
  odom->mutable_twist()->mutable_angular()->set_z(0.1);

  std::string serialized;
  micom.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Tx";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !odom_received; ++i) {
    tx_server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(odom_received) << "Did not receive Odometry message within timeout";

  EXPECT_EQ(received_odom.header.frame_id, "odom");
  EXPECT_EQ(received_odom.child_frame_id, "base_footprint");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  std::_Exit(result);
}
