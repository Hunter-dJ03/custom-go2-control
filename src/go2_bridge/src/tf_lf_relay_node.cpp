// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

/// Buffers non-leg dynamic transforms from /tf and republishes at a fixed rate on /go2/lf/tf.
/// Leg articulation for remote viz comes from /go2/lf/joint_states instead.
class TfLfRelayNode : public rclcpp::Node {
 public:
  TfLfRelayNode() : Node("tf_lf_relay_node") {
    input_topic_ = declare_parameter<std::string>("input_tf_topic", "/tf");
    output_topic_ = declare_parameter<std::string>("output_tf_topic", "/go2/lf/tf");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    const auto patterns = declare_parameter<std::vector<std::string>>(
        "exclude_child_frame_patterns", {"^(FR|FL|RR|RL)_"});

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "publish_rate_hz must be > 0");
      throw std::runtime_error("invalid publish_rate_hz");
    }

    for (const auto & pattern : patterns) {
      try {
        exclude_patterns_.emplace_back(pattern);
      } catch (const std::regex_error & e) {
        RCLCPP_ERROR(get_logger(), "Invalid exclude_child_frame_patterns regex '%s': %s",
                     pattern.c_str(), e.what());
        throw;
      }
    }

    const auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        input_topic_, tf_qos,
        std::bind(&TfLfRelayNode::onTf, this, std::placeholders::_1));

    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(output_topic_, tf_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&TfLfRelayNode::onTimer, this));

    RCLCPP_INFO(
        get_logger(), "Relaying %s -> %s at %.2f Hz (excluding %zu child-frame patterns)",
        input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_, exclude_patterns_.size());
  }

 private:
  static std::string transformKey(const geometry_msgs::msg::TransformStamped & t) {
    return t.header.frame_id + "|" + t.child_frame_id;
  }

  bool isExcludedChildFrame(const std::string & child_frame) const {
    for (const auto & pattern : exclude_patterns_) {
      if (std::regex_match(child_frame, pattern)) {
        return true;
      }
    }
    return false;
  }

  void onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & t : msg->transforms) {
      if (isExcludedChildFrame(t.child_frame_id)) {
        continue;
      }
      latest_[transformKey(t)] = t;
    }
  }

  void onTimer() {
    tf2_msgs::msg::TFMessage out;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      out.transforms.reserve(latest_.size());
      for (const auto & kv : latest_) {
        out.transforms.push_back(kv.second);
      }
    }
    if (out.transforms.empty()) {
      return;
    }
    tf_pub_->publish(out);
  }

  std::mutex mutex_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> latest_;
  std::vector<std::regex> exclude_patterns_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string input_topic_;
  std::string output_topic_;
  double publish_rate_hz_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfLfRelayNode>());
  rclcpp::shutdown();
  return 0;
}
