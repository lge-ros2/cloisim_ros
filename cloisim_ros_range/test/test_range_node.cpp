/**
 *  @file   test_range_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Range node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/sonar.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "cloisim_ros_range/range.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class RangeNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("sonar", "sonar_link"));
    data_port_ = server_->StartDataServer();

    ASSERT_GT(info_port_, 0);
    ASSERT_GT(data_port_, 0);
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    node_.reset();
    server_->Stop();
    server_.reset();
  }

  void CreateNode(const std::string & node_name = "test_range")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::Range>(options, node_name);
  }

  static cloisim::msgs::Sonar MakeSampleSonar()
  {
    cloisim::msgs::Sonar sonar;

    auto * stamp = sonar.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    sonar.set_frame("sonar_link");
    sonar.set_range_min(0.01);
    sonar.set_range_max(5.0);
    sonar.set_radius(0.3);
    sonar.set_range(1.5);

    auto * pose = sonar.mutable_world_pose();
    pose->mutable_position()->set_x(1.0);
    pose->mutable_position()->set_y(2.0);
    pose->mutable_position()->set_z(0.5);
    pose->mutable_orientation()->set_x(0.0);
    pose->mutable_orientation()->set_y(0.0);
    pose->mutable_orientation()->set_z(0.0);
    pose->mutable_orientation()->set_w(1.0);

    return sonar;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::Range> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(RangeNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(RangeNodeTest, PublishesRangeTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("sonar") != std::string::npos) {
      for (const auto & type : types) {
        if (type.find("Range") != std::string::npos) {
          found = true;
          break;
        }
      }
    }
    if (found) {break;}
  }
  EXPECT_TRUE(found) << "Expected Range topic on 'sonar' to be advertised";
}

TEST_F(RangeNodeTest, ReceivesAndPublishesRangeData)
{
  CreateNode();

  sensor_msgs::msg::Range received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_range_subscriber");

  // Range topic name: parts_name / topic_name
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Range>(
    "test_range/sonar", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Range::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto sonar = MakeSampleSonar();
  std::string serialized;
  sonar.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Data";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive Range message within timeout";

  EXPECT_NEAR(received_msg.range, 1.5, 0.001);
  EXPECT_NEAR(received_msg.min_range, 0.01, 0.001);
  EXPECT_NEAR(received_msg.max_range, 5.0, 0.001);
  EXPECT_NEAR(received_msg.field_of_view, 0.3, 0.001);  // radius = 0.3
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(result);
}
