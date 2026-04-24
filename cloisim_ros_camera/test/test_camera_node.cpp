/**
 *  @file   test_camera_node.cpp
 *  @date   2026-04-24
 *  @brief  Tests for cloisim_ros::Camera node
 *  @copyright SPDX-License-Identifier: MIT
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include <cloisim_msgs/image.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "cloisim_ros_camera/camera.hpp"
#include "mock_bridge_server.hpp"

using namespace std::chrono_literals;

class CameraNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<cloisim_ros::test::MockBridgeServer>();

    info_port_ = server_->StartInfoServerRaw(
      cloisim_ros::test::MakeCameraInfoHandler("camera", "camera_link", 320, 240));
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

  void CreateNode(const std::string & node_name = "test_camera")
  {
    auto options = cloisim_ros::test::MakeNodeOptions(info_port_, data_port_);
    node_ = std::make_shared<cloisim_ros::Camera>(options, node_name);
  }

  /// Build a sample Image protobuf message (RGB8, 4x2)
  static cloisim::msgs::Image MakeSampleImage(
    uint32_t width = 4, uint32_t height = 2)
  {
    cloisim::msgs::Image img;

    auto * stamp = img.mutable_header()->mutable_stamp();
    stamp->set_sec(100);
    stamp->set_nsec(0);

    img.set_width(width);
    img.set_height(height);
    img.set_pixel_format_type(cloisim::msgs::PixelFormatType::RGB_INT8);
    img.set_step(width * 3);

    // Fill with a simple pattern
    std::string pixel_data(width * height * 3, '\0');
    for (size_t i = 0; i < pixel_data.size(); i += 3) {
      pixel_data[i] = static_cast<char>(255);     // R
      pixel_data[i + 1] = static_cast<char>(0);   // G
      pixel_data[i + 2] = static_cast<char>(0);   // B
    }
    img.set_data(pixel_data);

    return img;
  }

  std::unique_ptr<cloisim_ros::test::MockBridgeServer> server_;
  std::shared_ptr<cloisim_ros::Camera> node_;
  uint16_t info_port_{0};
  uint16_t data_port_{0};
};

TEST_F(CameraNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NO_THROW(CreateNode());
  ASSERT_NE(node_, nullptr);
}

TEST_F(CameraNodeTest, PublishesImageTopic)
{
  CreateNode();

  auto topic_map = node_->get_topic_names_and_types();
  bool found = false;
  for (const auto & [name, types] : topic_map) {
    if (name.find("image_raw") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected 'image_raw' topic to be advertised";
}

TEST_F(CameraNodeTest, ReceivesAndPublishesImage)
{
  CreateNode();

  sensor_msgs::msg::Image received_msg;
  bool msg_received = false;

  auto sub_node = rclcpp::Node::make_shared("test_cam_subscriber");
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
    "test_camera/camera/image_raw", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Image::SharedPtr msg) {
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

  ASSERT_TRUE(msg_received) << "Did not receive Image message within timeout";

  EXPECT_EQ(received_msg.width, 4u);
  EXPECT_EQ(received_msg.height, 2u);
  EXPECT_EQ(received_msg.encoding, "rgb8");
  EXPECT_EQ(received_msg.step, 12u);
  EXPECT_EQ(received_msg.data.size(), 24u);
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
