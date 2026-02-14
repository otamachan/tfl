// Copyright 2026 Tamaki Nishino
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tfl/transform_buffer.hpp"
#include "tfl_ros/transform_listener.hpp"

class TransformListenerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ =
      std::make_shared<rclcpp::Node>("test_transform_listener_" + std::to_string(test_count_++));
  }

  void TearDown() override { node_.reset(); }

  rclcpp::Node::SharedPtr node_;
  static int test_count_;
};

int TransformListenerTest::test_count_ = 0;

TEST_F(TransformListenerTest, ReceiveDynamicTransform)
{
  tfl::TransformBuffer buffer;
  tfl_ros::TransformListener listener(buffer, node_);

  const auto pub = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));

  tf2_msgs::msg::TFMessage msg;
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = rclcpp::Time(1'000'000'000LL);
  ts.header.frame_id = "world";
  ts.child_frame_id = "base";
  ts.transform.rotation.w = 1.0;
  ts.transform.translation.x = 1.0;
  ts.transform.translation.y = 2.0;
  ts.transform.translation.z = 3.0;
  msg.transforms.push_back(ts);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    const auto result = buffer.lookup_transform("world", "base", 1'000'000'000LL);
    if (result.has_value()) {
      EXPECT_DOUBLE_EQ(result->translation[0], 1.0);
      EXPECT_DOUBLE_EQ(result->translation[1], 2.0);
      EXPECT_DOUBLE_EQ(result->translation[2], 3.0);
      EXPECT_DOUBLE_EQ(result->rotation[3], 1.0);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  FAIL() << "Transform not received within timeout";
}

TEST_F(TransformListenerTest, ReceiveMultipleTransforms)
{
  tfl::TransformBuffer buffer;
  tfl_ros::TransformListener listener(buffer, node_);

  const auto pub = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));

  tf2_msgs::msg::TFMessage msg;
  geometry_msgs::msg::TransformStamped ts1;
  ts1.header.stamp = rclcpp::Time(1'000'000'000LL);
  ts1.header.frame_id = "world";
  ts1.child_frame_id = "a";
  ts1.transform.rotation.w = 1.0;
  ts1.transform.translation.x = 1.0;

  geometry_msgs::msg::TransformStamped ts2;
  ts2.header.stamp = rclcpp::Time(1'000'000'000LL);
  ts2.header.frame_id = "a";
  ts2.child_frame_id = "b";
  ts2.transform.rotation.w = 1.0;
  ts2.transform.translation.y = 2.0;

  msg.transforms.push_back(ts1);
  msg.transforms.push_back(ts2);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    const auto result = buffer.lookup_transform("world", "b", 1'000'000'000LL);
    if (result.has_value()) {
      EXPECT_DOUBLE_EQ(result->translation[0], 1.0);
      EXPECT_DOUBLE_EQ(result->translation[1], 2.0);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  FAIL() << "Chained transform not received within timeout";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
