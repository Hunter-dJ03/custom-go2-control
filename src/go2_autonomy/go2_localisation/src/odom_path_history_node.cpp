// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

/// Subscribes to odometry and republishes the recent trajectory as nav_msgs/Path (sliding time window).
class OdomPathHistoryNode : public rclcpp::Node {
 public:
  OdomPathHistoryNode()
      : Node("odom_path_history_node"),
        horizon_(rclcpp::Duration::from_seconds(
            declare_parameter<double>("path_history_seconds", 120.0))) {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/go2/odom");
    path_topic_ = declare_parameter<std::string>("path_topic", "/go2/path");
    max_poses_ = declare_parameter<int>("max_path_poses", 50000);
    {
      const int s = declare_parameter<int>("path_odom_stride", 15);
      path_odom_stride_ = s < 1 ? 1 : s;
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).keep_last(1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomPathHistoryNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Path %s: publish when a pose is appended (stride=%d); window %.1f s, max_poses=%d",
                path_topic_.c_str(), path_odom_stride_, horizon_.seconds(), max_poses_);
  }

 private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ++odom_receive_count_;
    const bool appended = (odom_receive_count_ % path_odom_stride_ == 0);
    if (appended) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg->header;
      ps.pose = msg->pose.pose;
      poses_.push_back(std::move(ps));
    }

    const rclcpp::Time cutoff = now() - horizon_;
    while (!poses_.empty() && rclcpp::Time(poses_.front().header.stamp) < cutoff) {
      poses_.pop_front();
    }
    if (max_poses_ > 0) {
      while (static_cast<int>(poses_.size()) > max_poses_) {
        poses_.pop_front();
      }
    }

    if (!appended) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = msg->header.frame_id;
    path.poses.assign(poses_.begin(), poses_.end());
    path_pub_->publish(path);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::deque<geometry_msgs::msg::PoseStamped> poses_;
  rclcpp::Duration horizon_;
  std::string odom_topic_;
  std::string path_topic_;
  int max_poses_{0};
  int path_odom_stride_{15};
  uint32_t odom_receive_count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPathHistoryNode>());
  rclcpp::shutdown();
  return 0;
}
