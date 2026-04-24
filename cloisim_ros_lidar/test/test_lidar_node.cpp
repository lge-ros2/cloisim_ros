/**
 *  @file   test_lidar_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Lidar node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/header.pb.h>
#include <cloisim_msgs/laserscan.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "cloisim_ros_lidar/lidar.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class LidarNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("scan", "base_scan", "LaserScan"));
    data_port_ = server_->StartDataServer();

    ASSERT_GT(info_port_, 0);
    ASSERT_GT(data_port_, 0);

    // Small delay for server sockets to be ready
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    node_.reset();
    server_->Stop();
    server_.reset();
  }

  void CreateNode(const std::string & node_name = "test_lidar")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::Lidar>(options, node_name);
  }

  /// Build a sample LaserScan protobuf message
  static cloisim::msgs::LaserScan MakeSampleLaserScan(
    int beam_count = 10, double range_min = 0.1, double range_max = 30.0)
  {
    cloisim::msgs::LaserScan scan;

    auto * stamp = scan.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(500000000);

    scan.set_frame("base_scan");
    scan.set_angle_min(-1.5708);
    scan.set_angle_max(1.5708);
    scan.set_angle_step(3.1416 / beam_count);
    scan.set_range_min(range_min);
    scan.set_range_max(range_max);
    scan.set_count(beam_count);
    scan.set_vertical_angle_min(0.0);
    scan.set_vertical_angle_max(0.0);
    scan.set_vertical_angle_step(0.0);
    scan.set_vertical_count(1);

    for (int i = 0; i < beam_count; ++i) {
      scan.add_ranges(1.0 + 0.1 * i);
      scan.add_intensities(100.0);
    }

    return scan;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::Lidar> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(LidarNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(LidarNodeTest, PublishesScanTopic)
{
  CreateNode();

  // Check that the scan topic exists
  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("scan") != std::string::npos) {
      found = true;
      // Verify it's a LaserScan type
      for (const auto & t : types) {
        EXPECT_TRUE(t.find("LaserScan") != std::string::npos)
          << "Expected LaserScan type, got: " << t;
      }
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'scan' topic to be advertised";
}

TEST_F(LidarNodeTest, ReceivesAndPublishesLaserScan)
{
  CreateNode();

  // Create a subscriber to capture published messages
  sensor_msgs::msg::LaserScan received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_subscriber");
  auto sub = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  // Send test data through the mock server
  auto scan = MakeSampleLaserScan();
  std::string serialized;
  scan.SerializeToString(&serialized);

  // The hash key for data: node name (no namespace) + "Data"
  std::string hash_key = std::string(node_->get_name()) + "Data";

  // Give ZMQ SUB time to connect and subscribe
  std::this_thread::sleep_for(200ms);

  // Publish multiple times to ensure delivery (ZMQ PUB/SUB may drop first messages)
  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);

    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive LaserScan message within timeout";

  // Verify message content
  EXPECT_EQ(received_msg.header.frame_id, "base_scan");
  EXPECT_NEAR(received_msg.angle_min, -1.5708, 0.001);
  EXPECT_NEAR(received_msg.angle_max, 1.5708, 0.001);
  EXPECT_NEAR(received_msg.range_min, 0.1, 0.001);
  EXPECT_NEAR(received_msg.range_max, 30.0, 0.001);
  EXPECT_EQ(static_cast<int>(received_msg.ranges.size()), 10);
  EXPECT_NEAR(received_msg.ranges[0], 1.0, 0.001);
  EXPECT_NEAR(received_msg.ranges[9], 1.9, 0.001);
}

TEST_F(LidarNodeTest, PointCloud2OutputType)
{
  // Restart server with PointCloud2 output type
  server_->Stop();
  server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();
  info_port_ = server_->StartInfoServer(
    cloisim_ros::test::MakeDefaultInfoHandler("scan", "base_scan", "PointCloud2"));
  data_port_ = server_->StartDataServer();
  std::this_thread::sleep_for(50ms);

  CreateNode("test_lidar_pc2");

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("scan") != std::string::npos) {
      found = true;
      for (const auto & t : types) {
        EXPECT_TRUE(t.find("PointCloud2") != std::string::npos)
          << "Expected PointCloud2 type, got: " << t;
      }
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'scan' topic with PointCloud2 type";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  // Flush gtest output before _Exit
  std::fflush(stdout);
  std::fflush(stderr);
  // Use _Exit to avoid rclcpp atexit shutdown calling on_shutdown callbacks
  // on already-destroyed nodes (Base::Start registers rclcpp::on_shutdown
  // with a raw this pointer that becomes dangling after TearDown).
  std::_Exit(result);
}
