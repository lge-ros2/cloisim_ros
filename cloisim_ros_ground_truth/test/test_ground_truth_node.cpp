/**
 *  @file   test_ground_truth_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::GroundTruth node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/perception.pb.h>
#include <cloisim_msgs/perception_v.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/msg/object_array.hpp>

#include "cloisim_ros_ground_truth/ground_truth.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class GroundTruthNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    // GroundTruth only uses bridge.Data, no INFO server needed
    data_port_ = server_->StartDataServer();
    ASSERT_GT(data_port_, 0);
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    node_.reset();
    server_->Stop();
    server_.reset();
  }

  void CreateNode(const std::string & node_name = "test_ground_truth")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("bridge.Data", data_port_);
    // GroundTruth uses GetModelName() + GetPartsName() as hash key, not GetTargetHashKey
    options.append_parameter_override("model", std::string("test_model"));
    node_ = std::make_shared<cloisim_ros::GroundTruth>(options, node_name);
  }

  static cloisim::msgs::Perception_V MakeSamplePerceptions()
  {
    cloisim::msgs::Perception_V pv;

    auto * stamp = pv.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    auto * p1 = pv.add_perception();
    auto * p1_stamp = p1->mutable_header()->mutable_stamp();
    p1_stamp->set_sec(100);
    p1_stamp->set_nsec(0);
    p1->set_tracking_id(1);
    p1->set_class_id(3);
    p1->mutable_position()->set_x(5.0);
    p1->mutable_position()->set_y(2.0);
    p1->mutable_position()->set_z(0.0);
    p1->mutable_velocity()->set_x(1.0);
    p1->mutable_velocity()->set_y(0.0);
    p1->mutable_velocity()->set_z(0.0);
    p1->mutable_size()->set_x(0.5);
    p1->mutable_size()->set_y(0.5);
    p1->mutable_size()->set_z(1.8);

    auto * p2 = pv.add_perception();
    auto * p2_stamp = p2->mutable_header()->mutable_stamp();
    p2_stamp->set_sec(100);
    p2_stamp->set_nsec(0);
    p2->set_tracking_id(2);
    p2->set_class_id(1);
    p2->mutable_position()->set_x(10.0);
    p2->mutable_position()->set_y(-3.0);
    p2->mutable_position()->set_z(0.0);

    return pv;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::GroundTruth> node_;
  uint16_t data_port_{0};
};

TEST_F(GroundTruthNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(GroundTruthNodeTest, PublishesGroundTruthTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("ground_truth") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected '/ground_truth' topic to be advertised";
}

TEST_F(GroundTruthNodeTest, ReceivesAndPublishesPerceptionData)
{
  CreateNode();

  perception_msgs::msg::ObjectArray received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_gt_subscriber");
  auto sub = sub_node->create_subscription<perception_msgs::msg::ObjectArray>(
    "/ground_truth", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
    [&](const perception_msgs::msg::ObjectArray::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto pv = MakeSamplePerceptions();
  std::string serialized;
  pv.SerializeToString(&serialized);

  // GroundTruth hash key: GetModelName() + GetPartsName() + "Data"
  std::string hash_key = std::string("test_model") + std::string(node_->get_name()) + "Data";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive ObjectArray message within timeout";

  ASSERT_EQ(received_msg.objects.size(), 2u);
  EXPECT_EQ(received_msg.objects[0].tracking_id, 1);
  EXPECT_EQ(received_msg.objects[0].class_id, 3);
  EXPECT_NEAR(received_msg.objects[0].position.x, 5.0, 0.001);
  EXPECT_EQ(received_msg.objects[1].tracking_id, 2);
  EXPECT_EQ(received_msg.objects[1].class_id, 1);
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
