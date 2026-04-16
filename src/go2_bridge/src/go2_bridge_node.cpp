// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "go2_interfaces/srv/sport_api_call.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace {

constexpr std::array<const char *, 12> kLegJointNames = {
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint",
  "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint",
  "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

}  // namespace

class Go2BridgeNode : public rclcpp::Node {
 public:
  Go2BridgeNode()
      : Node("go2_bridge_node"),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    sport_topic_ =
        declare_parameter<std::string>("sport_mode_state_topic", "/sportmodestate");
    low_topic_ = declare_parameter<std::string>("low_state_topic", "/lowstate");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/go2/odom", rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/go2/imu", rclcpp::SensorDataQoS());
    joint_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/go2/joint_states", rclcpp::SensorDataQoS());
    battery_pub_ =
        create_publisher<sensor_msgs::msg::BatteryState>("/go2/battery", rclcpp::QoS(10));

    sport_api_pub_ = create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", rclcpp::QoS(10));

    sport_srv_ = create_service<go2_interfaces::srv::SportApiCall>(
        "/go2/sport_api_call",
        std::bind(&Go2BridgeNode::onSportApiCall, this, std::placeholders::_1, std::placeholders::_2));

    sport_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        sport_topic_, rclcpp::SensorDataQoS(),
        std::bind(&Go2BridgeNode::onSportModeState, this, std::placeholders::_1));

    low_sub_ = create_subscription<unitree_go::msg::LowState>(
        low_topic_, rclcpp::SensorDataQoS(),
        std::bind(&Go2BridgeNode::onLowState, this, std::placeholders::_1));

    sport_resp_sub_ = create_subscription<unitree_api::msg::Response>(
        "/api/sport/response", rclcpp::QoS(10),
        std::bind(&Go2BridgeNode::onSportResponse, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "go2_bridge_node listening on %s, %s", sport_topic_.c_str(),
                low_topic_.c_str());
  }

 private:
  void onSportModeState(const unitree_go::msg::SportModeState::SharedPtr msg) {
    (void)msg->stamp;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];

    // Unitree IMUState.quaternion is [w, x, y, z]
    odom.pose.pose.orientation.w = msg->imu_state.quaternion[0];
    odom.pose.pose.orientation.x = msg->imu_state.quaternion[1];
    odom.pose.pose.orientation.y = msg->imu_state.quaternion[2];
    odom.pose.pose.orientation.z = msg->imu_state.quaternion[3];

    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    odom.twist.twist.linear.z = msg->velocity[2];
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = msg->yaw_speed;

    // Covariance unknown
    for (double &c : odom.pose.covariance) {
      c = 0.0;
    }
    odom.pose.covariance[0] = -1.0;
    for (double &c : odom.twist.covariance) {
      c = 0.0;
    }
    odom.twist.covariance[0] = -1.0;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = odom.header.stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void onLowState(const unitree_go::msg::LowState::SharedPtr msg) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = base_frame_;
    imu.orientation.w = msg->imu_state.quaternion[0];
    imu.orientation.x = msg->imu_state.quaternion[1];
    imu.orientation.y = msg->imu_state.quaternion[2];
    imu.orientation.z = msg->imu_state.quaternion[3];
    imu.angular_velocity.x = msg->imu_state.gyroscope[0];
    imu.angular_velocity.y = msg->imu_state.gyroscope[1];
    imu.angular_velocity.z = msg->imu_state.gyroscope[2];
    imu.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imu.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imu.linear_acceleration.z = msg->imu_state.accelerometer[2];
    imu_pub_->publish(imu);

    sensor_msgs::msg::JointState js;
    js.header.stamp = imu.header.stamp;
    js.name.assign(kLegJointNames.begin(), kLegJointNames.end());
    js.position.resize(12);
    js.velocity.resize(12);
    js.effort.resize(12);
    for (size_t i = 0; i < 12; ++i) {
      const auto &m = msg->motor_state[i];
      js.position[i] = m.q;
      js.velocity[i] = m.dq;
      js.effort[i] = m.tau_est;
    }
    joint_pub_->publish(js);

    ++low_state_count_;
    if (low_state_count_ >= battery_publish_stride_) {
      low_state_count_ = 0;
      sensor_msgs::msg::BatteryState bat;
      bat.header.stamp = imu.header.stamp;
      bat.voltage = msg->power_v;
      // BMS current is typically reported in mA
      bat.current = static_cast<float>(msg->bms_state.current) * 0.001f;
      bat.percentage = static_cast<float>(msg->bms_state.soc);
      bat.present = true;
      bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      bat.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
      battery_pub_->publish(bat);
    }
  }

  void onSportResponse(const unitree_api::msg::Response::SharedPtr msg) {
    const int32_t api = static_cast<int32_t>(msg->header.identity.api_id);
    const int32_t code = msg->header.status.code;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/api/sport/response: api_id=%d status.code=%d (data prefix: %.80s)", api, code,
        msg->data.c_str());
  }

  void onSportApiCall(const std::shared_ptr<go2_interfaces::srv::SportApiCall::Request> req,
                      std::shared_ptr<go2_interfaces::srv::SportApiCall::Response> res) {
    unitree_api::msg::Request ureq;
    ureq.header.identity.api_id = req->api_id;
    ureq.header.identity.id = 0;
    ureq.parameter = req->parameter_json;
    try {
      sport_api_pub_->publish(ureq);
    } catch (const std::exception &e) {
      res->success = false;
      res->message = std::string("publish failed: ") + e.what();
      return;
    }
    res->success = true;
    res->message = "published to /api/sport/request";
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_api_pub_;

  rclcpp::Service<go2_interfaces::srv::SportApiCall>::SharedPtr sport_srv_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr sport_resp_sub_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string sport_topic_;
  std::string low_topic_;

  uint32_t low_state_count_{0};
  uint32_t battery_publish_stride_{25};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
