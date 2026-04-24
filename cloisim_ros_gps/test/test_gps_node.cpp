/**
 *  @file   test_gps_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Gps node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cloisim_msgs/navsat_with_covariance.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "cloisim_ros_gps/gps.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class GpsNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServer(
      cloisim_ros::test::MakeDefaultInfoHandler("gps", "gps_link"));
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

  void CreateNode(const std::string & node_name = "test_gps")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::Gps>(options, node_name);
  }

  static cloisim::msgs::NavSatWithCovariance MakeSampleNavSat()
  {
    cloisim::msgs::NavSatWithCovariance navsat;

    auto * stamp = navsat.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    navsat.set_frame_id("gps_link");
    navsat.set_latitude_deg(37.3861);
    navsat.set_longitude_deg(-122.0839);
    navsat.set_altitude(10.0);
    navsat.set_velocity_east(1.0);
    navsat.set_velocity_north(2.0);
    navsat.set_velocity_up(0.0);

    return navsat;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::Gps> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(GpsNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(GpsNodeTest, PublishesNavSatFixTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("gps") != std::string::npos || name.find("navsatfix") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected GPS-related topic to be advertised";
}

TEST_F(GpsNodeTest, ReceivesAndPublishesGpsData)
{
  CreateNode();

  sensor_msgs::msg::NavSatFix received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_gps_subscriber");
  auto sub = sub_node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "gps", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      received_msg = *msg;
      msg_received = true;
    });

  auto navsat = MakeSampleNavSat();
  std::string serialized;
  navsat.SerializeToString(&serialized);

  std::string hash_key = std::string(node_->get_name()) + "Data";

  std::this_thread::sleep_for(200ms);

  for (int i = 0; i < 10 && !msg_received; ++i) {
    server_->PublishData(hash_key, serialized);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_);
    rclcpp::spin_some(sub_node);
  }

  ASSERT_TRUE(msg_received) << "Did not receive NavSatFix message within timeout";

  EXPECT_NEAR(received_msg.latitude, 37.3861, 0.001);
  EXPECT_NEAR(received_msg.longitude, -122.0839, 0.001);
  EXPECT_NEAR(received_msg.altitude, 10.0, 0.1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  std::_Exit(result);
}
