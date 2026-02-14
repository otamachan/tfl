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

#include "tfl_ros/transform_listener.hpp"

#include <utility>

namespace tfl_ros
{

TransformListener::TransformListener(
  tfl::TransformBuffer & buffer, rclcpp::Node::SharedPtr node, bool static_only)
: buffer_(buffer), node_(std::move(node))
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  sub_tf_static_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local(),
    [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg) {
      subscription_callback(msg, true);
    },
    options);

  if (!static_only) {
    sub_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg) {
        subscription_callback(msg, false);
      },
      options);
  }

  auto post_jump_cb = [this](const rcl_time_jump_t & jump) {
    if (
      RCL_ROS_TIME_ACTIVATED == jump.clock_change ||
      RCL_ROS_TIME_DEACTIVATED == jump.clock_change || jump.delta.nanoseconds < 0) {
      buffer_.clear();
    }
  };
  rcl_jump_threshold_t threshold;
  threshold.min_forward.nanoseconds = 0;
  threshold.min_backward.nanoseconds = -1;
  threshold.on_clock_change = true;
  jump_handler_ = node_->get_clock()->create_jump_callback(nullptr, post_jump_cb, threshold);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
  dedicated_listener_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

TransformListener::~TransformListener()
{
  executor_->cancel();
  dedicated_listener_thread_->join();
  jump_handler_.reset();
}

void TransformListener::subscription_callback(
  const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg, bool is_static)
{
  for (const auto & ts : msg->transforms) {
    tfl::TransformData data;
    data.rotation = {
      ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z,
      ts.transform.rotation.w};
    data.translation = {
      ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z};
    data.stamp_ns = rclcpp::Time(ts.header.stamp).nanoseconds();
    buffer_.set_transform(ts.child_frame_id, ts.header.frame_id, data, is_static);
  }
}

}  // namespace tfl_ros
