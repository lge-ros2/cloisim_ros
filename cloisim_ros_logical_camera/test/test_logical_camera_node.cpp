/**
 *  @file   test_logical_camera_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::LogicalCamera node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/logical_camera_image.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "cloisim_ros_logical_camera/logical_camera.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class LogicalCameraNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("logical_camera", "logical_camera_link"));
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

  void CreateNode(const std::string & node_name = "test_logical_camera")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::LogicalCamera>(options, node_name);
  }

  static cloisim::msgs::LogicalCameraImage MakeSampleImage()
  {
    cloisim::msgs::LogicalCameraImage img;

    auto * stamp = img.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    auto * pose = img.mutable_pose();
    pose->mutable_position()->set_x(0.0);
    pose->mutable_position()->set_y(0.0);
    pose->mutable_position()->set_z(2.0);
    pose->mutable_orientation()->set_x(0.0);
    pose->mutable_orientation()->set_y(0.0);
    pose->mutable_orientation()->set_z(0.0);
    pose->mutable_orientation()->set_w(1.0);

    auto * m1 = img.add_model();
    m1->set_name("box_01");
    m1->mutable_pose()->mutable_position()->set_x(1.0);
    m1->mutable_pose()->mutable_position()->set_y(2.0);
    m1->mutable_pose()->mutable_position()->set_z(0.0);
    m1->mutable_pose()->mutable_orientation()->set_x(0.0);
    m1->mutable_pose()->mutable_orientation()->set_y(0.0);
    m1->mutable_pose()->mutable_orientation()->set_z(0.0);
    m1->mutable_pose()->mutable_orientation()->set_w(1.0);

    auto * m2 = img.add_model();
    m2->set_name("cylinder_02");
    m2->mutable_pose()->mutable_position()->set_x(3.0);
    m2->mutable_pose()->mutable_position()->set_y(-1.0);
    m2->mutable_pose()->mutable_position()->set_z(0.5);
    m2->mutable_pose()->mutable_orientation()->set_x(0.0);
    m2->mutable_pose()->mutable_orientation()->set_y(0.0);
    m2->mutable_pose()->mutable_orientation()->set_z(0.0);
    m2->mutable_pose()->mutable_orientation()->set_w(1.0);

    return img;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::LogicalCamera> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(LogicalCameraNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(LogicalCameraNodeTest, PublishesLogicalCameraTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("logical_camera") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'logical_camera' topic to be advertised";
}

TEST_F(LogicalCameraNodeTest, ReceivesAndPublishesDetections)
{
  CreateNode();

  vision_msgs::msg::Detection3DArray received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_logical_camera_subscriber");
  auto sub = sub_node->create_subscription<vision_msgs::msg::Detection3DArray>(
    "logical_camera", rclcpp::SensorDataQoS(),
    [&](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto img = MakeSampleImage();
  std::string serialized;
  img.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Data";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive Detection3DArray message within timeout";

  ASSERT_EQ(received_msg.detections.size(), 2u);

  ASSERT_FALSE(received_msg.detections[0].results.empty());
  EXPECT_EQ(received_msg.detections[0].results[0].hypothesis.class_id, "box_01");
  EXPECT_NEAR(received_msg.detections[0].results[0].hypothesis.score, 1.0, 0.001);
  EXPECT_NEAR(received_msg.detections[0].results[0].pose.pose.position.x, 1.0, 0.001);
  EXPECT_NEAR(received_msg.detections[0].results[0].pose.pose.position.y, 2.0, 0.001);

  ASSERT_FALSE(received_msg.detections[1].results.empty());
  EXPECT_EQ(received_msg.detections[1].results[0].hypothesis.class_id, "cylinder_02");
  EXPECT_NEAR(received_msg.detections[1].results[0].pose.pose.position.x, 3.0, 0.001);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  std::_Exit(result);
}
