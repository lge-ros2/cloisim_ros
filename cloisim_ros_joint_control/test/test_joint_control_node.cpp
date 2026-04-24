/**
 *  @file   test_joint_control_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::JointControl node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/joint_state.pb.h>
#include <cloisim_msgs/joint_state_v.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include "cloisim_ros_joint_control/joint_control.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class JointControlNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();
    tx_server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("", "base_link"));
    tx_port_ = tx_server_->StartDataServer();

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

  void CreateNode(const std::string & node_name = "test_joint_control")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("bridge.Info", info_port_);
    options.append_parameter_override("bridge.Tx", tx_port_);
    options.append_parameter_override("bridge.Rx", uint16_t(0));
    options.append_parameter_override("bridge.Tf", uint16_t(0));
    node_ = std::make_shared<cloisim_ros::JointControl>(options, node_name);
  }

  static cloisim::msgs::JointState_V MakeSampleJointStates()
  {
    cloisim::msgs::JointState_V jsv;

    auto * stamp = jsv.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    auto * j1 = jsv.add_joint_state();
    j1->set_name("joint_1");
    j1->set_position(0.5);
    j1->set_velocity(0.1);
    j1->set_effort(1.0);

    auto * j2 = jsv.add_joint_state();
    j2->set_name("joint_2");
    j2->set_position(-0.3);
    j2->set_velocity(0.0);
    j2->set_effort(0.5);

    return jsv;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::unique_ptr<cloisim_ros::test::MockBridgeServer> tx_server_;
  std::shared_ptr<cloisim_ros::JointControl> node_;
  uint16_t info_port_{0};
  uint16_t tx_port_{0};
};

TEST_F(JointControlNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(JointControlNodeTest, PublishesExpectedTopics)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();

  bool found_joint_states = false;
  bool found_robot_desc = false;

  for (const auto & [name, types] : topic_map) {
    if (name.find("joint_states") != std::string::npos) {found_joint_states = true;}
    if (name.find("robot_description") != std::string::npos) {found_robot_desc = true;}
  }

  EXPECT_TRUE(found_joint_states) << "Expected 'joint_states' topic";
  EXPECT_TRUE(found_robot_desc) << "Expected 'robot_description' topic";
}

TEST_F(JointControlNodeTest, SubscribesToJointCommand)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("joint_command") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'joint_command' subscription";
}

TEST_F(JointControlNodeTest, ReceivesAndPublishesJointStates)
{
  CreateNode();

  sensor_msgs::msg::JointState received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_jc_subscriber");
  auto sub = sub_node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto jsv = MakeSampleJointStates();
  std::string serialized;
  jsv.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Tx";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    tx_server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive JointState message within timeout";

  ASSERT_EQ(received_msg.name.size(), 2u);
  EXPECT_EQ(received_msg.name[0], "joint_1");
  EXPECT_EQ(received_msg.name[1], "joint_2");
  EXPECT_NEAR(received_msg.position[0], 0.5, 0.001);
  EXPECT_NEAR(received_msg.position[1], -0.3, 0.001);
  EXPECT_NEAR(received_msg.velocity[0], 0.1, 0.001);
  EXPECT_NEAR(received_msg.effort[0], 1.0, 0.001);
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
