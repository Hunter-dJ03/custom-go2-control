// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0
//
// Subscribes to geometry_msgs/Twist (SI: m/s and rad/s) and sends Sport Move
// (API 1008) by publishing unitree_api::msg::Request on /api/sport/request
// (same as Unitree examples). No go2_interfaces dependency — avoids mixed
// workspace typesupport symbol errors at dynamic load.
//
// zero_velocity_debounce_sec: brief all-zero cmd_vel bursts (e.g. teleop gaps)
// are ignored and the previous non-zero command is held for this duration.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

namespace {

constexpr int32_t kSportApiMove = 1008;

bool TwistAllZero(double lx, double ly, double az, double eps = 1e-6) {
  return std::abs(lx) < eps && std::abs(ly) < eps && std::abs(az) < eps;
}

}  // namespace

class CmdVelToSportNode : public rclcpp::Node {
 public:
  CmdVelToSportNode()
      : Node("cmd_vel_to_sport_node") {
    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/go2/teleop/cmd_vel");
    send_hz_ = declare_parameter<double>("send_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.65);
    zero_velocity_debounce_sec_ = declare_parameter<double>("zero_velocity_debounce_sec", 0.12);
    max_linear_x_ = declare_parameter<double>("max_linear_velocity_x", 1.0);
    max_linear_y_ = declare_parameter<double>("max_linear_velocity_y", 0.5);
    max_angular_z_ = declare_parameter<double>("max_angular_velocity_z", 1.5);
    log_every_n_ = declare_parameter<int>("log_every_n_moves", 0);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic_, rclcpp::QoS(10),
        std::bind(&CmdVelToSportNode::onCmdVel, this, std::placeholders::_1));

    sport_req_pub_ = create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(send_hz_, 1.0));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CmdVelToSportNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
                "cmd_vel_to_sport: in=%s | Move(1008) -> /api/sport/request "
                "(Twist: linear m/s, angular rad/s); zero_debounce=%.3fs",
                cmd_topic_.c_str(), zero_velocity_debounce_sec_);
  }

 private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_linear_x_ = msg->linear.x;
    last_linear_y_ = msg->linear.y;
    last_angular_z_ = msg->angular.z;
    have_cmd_ = true;
    last_cmd_time_ = now();

    if (!TwistAllZero(last_linear_x_, last_linear_y_, last_angular_z_)) {
      zero_since_valid_ = false;
      last_nz_linear_x_ = last_linear_x_;
      last_nz_linear_y_ = last_linear_y_;
      last_nz_angular_z_ = last_angular_z_;
    } else {
      if (!zero_since_valid_) {
        zero_since_ = now();
        zero_since_valid_ = true;
      }
    }
  }

  void onTimer() {
    const rclcpp::Time tnow = now();
    if (!have_cmd_ || (tnow - last_cmd_time_).seconds() > command_timeout_sec_) {
      zero_since_valid_ = false;
      publishMove(0.0, 0.0, 0.0);
      return;
    }

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    if (!TwistAllZero(last_linear_x_, last_linear_y_, last_angular_z_)) {
      vx = last_linear_x_;
      vy = last_linear_y_;
      wz = last_angular_z_;
    } else {
      if (zero_since_valid_ &&
          (tnow - zero_since_).seconds() < zero_velocity_debounce_sec_ &&
          !TwistAllZero(last_nz_linear_x_, last_nz_linear_y_, last_nz_angular_z_)) {
        vx = last_nz_linear_x_;
        vy = last_nz_linear_y_;
        wz = last_nz_angular_z_;
      } else {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
      }
    }

    vx = std::clamp(vx, -std::abs(max_linear_x_), std::abs(max_linear_x_));
    vy = std::clamp(vy, -std::abs(max_linear_y_), std::abs(max_linear_y_));
    wz = std::clamp(wz, -std::abs(max_angular_z_), std::abs(max_angular_z_));
    publishMove(vx, vy, wz);
  }

  void publishMove(double vx, double vy, double wz) {
    nlohmann::json js;
    js["x"] = static_cast<float>(vx);
    js["y"] = static_cast<float>(vy);
    js["z"] = static_cast<float>(wz);

    unitree_api::msg::Request ureq;
    ureq.header.identity.api_id = kSportApiMove;
    ureq.header.identity.id = 0;
    ureq.parameter = js.dump();
    sport_req_pub_->publish(ureq);

    ++move_count_;
    if (log_every_n_ > 0 && (move_count_ % static_cast<uint64_t>(log_every_n_)) == 0U) {
      RCLCPP_INFO(get_logger(), "Move vx=%.3f vy=%.3f wz=%.3f (count=%lu)", vx, vy, wz,
                  static_cast<unsigned long>(move_count_));
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_req_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string cmd_topic_;
  double send_hz_{50.0};
  double command_timeout_sec_{0.65};
  double zero_velocity_debounce_sec_{0.12};
  double max_linear_x_{1.0};
  double max_linear_y_{0.5};
  double max_angular_z_{1.5};
  int log_every_n_{0};

  bool have_cmd_{false};
  rclcpp::Time last_cmd_time_;
  double last_linear_x_{0.0};
  double last_linear_y_{0.0};
  double last_angular_z_{0.0};

  bool zero_since_valid_{false};
  rclcpp::Time zero_since_;
  double last_nz_linear_x_{0.0};
  double last_nz_linear_y_{0.0};
  double last_nz_angular_z_{0.0};

  uint64_t move_count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToSportNode>());
  rclcpp::shutdown();
  return 0;
}
