// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

/// Publishes map -> odom as a static transform. Placeholder until SLAM / localisation
/// provides a time-varying map -> odom correction.
class MapOdomTfNode : public rclcpp::Node {
 public:
  MapOdomTfNode() : Node("map_odom_tf_node") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

    tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_static_->sendTransform(t);

    RCLCPP_INFO(get_logger(), "Static TF %s -> %s (identity)", map_frame_.c_str(),
                odom_frame_.c_str());
  }

 private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;
  std::string map_frame_;
  std::string odom_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
