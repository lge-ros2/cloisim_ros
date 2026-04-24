/**
 *  @file   test_contact_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Contact node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/contacts.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

#include "cloisim_ros_contact/contact.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class ContactNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("contacts", "contact_link"));
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

  void CreateNode(const std::string & node_name = "test_contact")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::Contact>(options, node_name);
  }

  static cloisim::msgs::Contacts MakeSampleContacts()
  {
    cloisim::msgs::Contacts contacts;

    auto * stamp = contacts.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    auto * contact = contacts.add_contact();
    contact->mutable_collision1()->set_name("link1::collision");
    contact->mutable_collision2()->set_name("link2::collision");

    auto * pos = contact->add_position();
    pos->set_x(1.0);
    pos->set_y(0.0);
    pos->set_z(0.0);

    auto * normal = contact->add_normal();
    normal->set_x(0.0);
    normal->set_y(0.0);
    normal->set_z(1.0);

    contact->add_depth(0.01);

    auto * wrench = contact->add_wrench();
    wrench->set_body_1_name("link1");
    wrench->set_body_2_name("link2");
    wrench->set_body_1_id(1);
    wrench->set_body_2_id(2);
    wrench->mutable_body_1_wrench()->mutable_force()->set_x(10.0);
    wrench->mutable_body_1_wrench()->mutable_force()->set_y(0.0);
    wrench->mutable_body_1_wrench()->mutable_force()->set_z(0.0);
    wrench->mutable_body_2_wrench()->mutable_force()->set_x(-10.0);
    wrench->mutable_body_2_wrench()->mutable_force()->set_y(0.0);
    wrench->mutable_body_2_wrench()->mutable_force()->set_z(0.0);

    return contacts;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::Contact> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(ContactNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(ContactNodeTest, PublishesContactsTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("contacts") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected contacts topic to be advertised";
}

TEST_F(ContactNodeTest, ReceivesAndPublishesContactData)
{
  CreateNode();

  gazebo_msgs::msg::ContactsState received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_contact_subscriber");

  // Find the actual topic name
  std::string actual_topic;
  auto topic_map = node_->get_topic_names_and_types();
  for (const auto & [name, types] : topic_map) {
    if (name.find("contacts") != std::string::npos) {
      actual_topic = name;
      break;
    }
  }
  ASSERT_FALSE(actual_topic.empty()) << "Could not find contacts topic";

  auto sub = sub_node->create_subscription<gazebo_msgs::msg::ContactsState>(
    actual_topic, rclcpp::SensorDataQoS(),
    [&](const gazebo_msgs::msg::ContactsState::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto contacts = MakeSampleContacts();
  std::string serialized;
  contacts.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Data";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive Contacts message within timeout";

  ASSERT_EQ(received_msg.states.size(), 1u);
  EXPECT_EQ(received_msg.states[0].collision1_name, "link1::collision");
  EXPECT_EQ(received_msg.states[0].collision2_name, "link2::collision");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  std::_Exit(result);
}
