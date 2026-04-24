/**
 *  @file   test_imu_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Imu node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/imu.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "cloisim_ros_imu/imu.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class ImuNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("imu/data_raw", "imu_link"));
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

  void CreateNode(const std::string & node_name = "test_imu")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::Imu>(options, node_name);
  }

  static cloisim::msgs::IMU MakeSampleImu()
  {
    cloisim::msgs::IMU imu;

    auto * stamp = imu.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    imu.mutable_orientation()->set_x(0.0);
    imu.mutable_orientation()->set_y(0.0);
    imu.mutable_orientation()->set_z(0.0);
    imu.mutable_orientation()->set_w(1.0);

    imu.mutable_angular_velocity()->set_x(0.1);
    imu.mutable_angular_velocity()->set_y(0.2);
    imu.mutable_angular_velocity()->set_z(0.3);

    imu.mutable_linear_acceleration()->set_x(0.0);
    imu.mutable_linear_acceleration()->set_y(0.0);
    imu.mutable_linear_acceleration()->set_z(9.81);

    return imu;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::Imu> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(ImuNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(ImuNodeTest, PublishesImuTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("imu") != std::string::npos) {
      found = true;
      for (const auto & t : types) {
        EXPECT_TRUE(t.find("Imu") != std::string::npos)
          << "Expected Imu type, got: " << t;
      }
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'imu' topic to be advertised";
}

TEST_F(ImuNodeTest, ReceivesAndPublishesImuData)
{
  CreateNode();

  sensor_msgs::msg::Imu received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_imu_subscriber");
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto imu = MakeSampleImu();
  std::string serialized;
  imu.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Data";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive IMU message within timeout";

  EXPECT_EQ(received_msg.header.frame_id, "imu_link");
  EXPECT_NEAR(received_msg.orientation.w, 1.0, 0.001);
  EXPECT_NEAR(received_msg.angular_velocity.x, 0.1, 0.001);
  EXPECT_NEAR(received_msg.angular_velocity.y, 0.2, 0.001);
  EXPECT_NEAR(received_msg.angular_velocity.z, 0.3, 0.001);
  EXPECT_NEAR(received_msg.linear_acceleration.z, 9.81, 0.01);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  std::_Exit(result);
}
