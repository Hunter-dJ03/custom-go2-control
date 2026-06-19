// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/// Relays the latest JointState from the bridge at a fixed rate for remote visualization.
class JointStatesThrottleNode : public rclcpp::Node {
 public:
  JointStatesThrottleNode() : Node("joint_states_throttle_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/go2/joint_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "/go2/lf/joint_states");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "publish_rate_hz must be > 0");
      throw std::runtime_error("invalid publish_rate_hz");
    }

    const auto qos = rclcpp::SensorDataQoS();

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, qos,
        std::bind(&JointStatesThrottleNode::onJointState, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&JointStatesThrottleNode::onTimer, this));

    RCLCPP_INFO(
        get_logger(), "Relaying %s -> %s at %.2f Hz", input_topic_.c_str(), output_topic_.c_str(),
        publish_rate_hz_);
  }

 private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = *msg;
  }

  void onTimer() {
    std::optional<sensor_msgs::msg::JointState> msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_.has_value()) {
        return;
      }
      msg = latest_;
    }
    pub_->publish(*msg);
  }

  std::mutex mutex_;
  std::optional<sensor_msgs::msg::JointState> latest_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string input_topic_;
  std::string output_topic_;
  double publish_rate_hz_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatesThrottleNode>());
  rclcpp::shutdown();
  return 0;
}
