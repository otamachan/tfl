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

#pragma once

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tfl/transform_buffer.hpp"

namespace tfl_ros
{

class TransformListener
{
public:
  explicit TransformListener(
    tfl::TransformBuffer & buffer, rclcpp::Node::SharedPtr node, bool static_only = false);

  ~TransformListener();

  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;

private:
  void subscription_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg, bool is_static);

  tfl::TransformBuffer & buffer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<std::thread> dedicated_listener_thread_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};

}  // namespace tfl_ros
