// Copyright 2026 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace {

bool isFinite(double v) { return std::isfinite(v); }

bool isValidQuaternion(const geometry_msgs::msg::Quaternion & q) {
  return isFinite(q.x) && isFinite(q.y) && isFinite(q.z) && isFinite(q.w);
}

bool isValidPose(const geometry_msgs::msg::Pose & pose) {
  return isFinite(pose.position.x) && isFinite(pose.position.y) &&
         isFinite(pose.position.z) && isValidQuaternion(pose.orientation);
}

geometry_msgs::msg::Quaternion normalizedQuaternion(
    const geometry_msgs::msg::Quaternion & q) {
  geometry_msgs::msg::Quaternion out = q;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-9) {
    out.x = 0.0;
    out.y = 0.0;
    out.z = 0.0;
    out.w = 1.0;
    return out;
  }
  out.x /= norm;
  out.y /= norm;
  out.z /= norm;
  out.w /= norm;
  return out;
}

geometry_msgs::msg::Vector3 rotateVector(
    const geometry_msgs::msg::Quaternion & q,
    const geometry_msgs::msg::Vector3 & v) {
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  tf2::Matrix3x3 rotation(tf_q);
  tf2::Vector3 in(v.x, v.y, v.z);
  const tf2::Vector3 out = rotation * in;
  geometry_msgs::msg::Vector3 rotated;
  rotated.x = out.x();
  rotated.y = out.y();
  rotated.z = out.z();
  return rotated;
}

}  // namespace

class OdomRebaseNode : public rclcpp::Node {
 public:
  OdomRebaseNode()
      : Node("odom_rebase_node"),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    raw_odom_topic_ = declare_parameter<std::string>("raw_odom_topic", "odom_rgbd_raw");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/go2/odom");
    seed_pose_topic_ = declare_parameter<std::string>("seed_pose_topic", "/go2/odom_seed");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    // Keep sensor timestamps for ZED/visual sync; enable host time only for LiDAR deskewing.
    use_host_time_ = declare_parameter<bool>("use_host_time", false);
    transform_input_to_base_ = declare_parameter<bool>("transform_input_to_base", false);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.2);

    if (transform_input_to_base_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    output_odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::SensorDataQoS());

    seed_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        seed_pose_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&OdomRebaseNode::onSeedPose, this, std::placeholders::_1));

    raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        raw_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomRebaseNode::onRawOdom, this, std::placeholders::_1));
  }

 private:
  bool transformPoseToBase(
      const nav_msgs::msg::Odometry & msg,
      geometry_msgs::msg::Pose & pose_out,
      geometry_msgs::msg::Twist & twist_out) {
    pose_out = msg.pose.pose;
    twist_out = msg.twist.twist;

    if (!transform_input_to_base_ || msg.child_frame_id == base_frame_) {
      return true;
    }

    if (!tf_buffer_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "transform_input_to_base is true but TF buffer is unavailable");
      return false;
    }

    try {
      const geometry_msgs::msg::TransformStamped child_to_base = tf_buffer_->lookupTransform(
          base_frame_, msg.child_frame_id, msg.header.stamp,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));

      tf2::Transform odom_to_child;
      tf2::fromMsg(msg.pose.pose, odom_to_child);

      tf2::Transform child_to_base_tf;
      tf2::fromMsg(child_to_base.transform, child_to_base_tf);

      tf2::Transform odom_to_base = odom_to_child * child_to_base_tf;
      tf2::toMsg(odom_to_base, pose_out);

      twist_out.linear = rotateVector(child_to_base.transform.rotation, msg.twist.twist.linear);
      twist_out.angular = rotateVector(child_to_base.transform.rotation, msg.twist.twist.angular);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform odometry from %s to %s: %s",
          msg.child_frame_id.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void publishTransform(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = normalizedQuaternion(pose.orientation);
    tf_broadcaster_->sendTransform(tf);
    last_good_pose_ = pose;
    has_last_good_pose_ = true;
  }

  void onSeedPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!isValidPose(msg->pose)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid odom seed pose on %s", seed_pose_topic_.c_str());
      return;
    }

    seed_pose_ = msg->pose;
    seed_z_ = msg->pose.position.z;
    has_seed_pose_ = true;
    RCLCPP_INFO_ONCE(
        get_logger(), "Captured odom seed pose from %s with startup z=%.3f m",
        seed_pose_topic_.c_str(), seed_z_);

    // Bootstrap a valid odom->base_link TF at startup height before odometry runs.
    if (publish_tf_) {
      publishTransform(msg->header.stamp, seed_pose_);
    }
  }

  void onRawOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!has_seed_pose_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for startup odom seed on %s before publishing rebased odometry",
          seed_pose_topic_.c_str());
      return;
    }

    if (!isValidPose(msg->pose.pose)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping invalid raw odometry (tracking lost); holding last good odom/TF");
      if (publish_tf_ && has_last_good_pose_) {
        const rclcpp::Time stamp =
            use_host_time_ ? now() : rclcpp::Time(msg->header.stamp);
        publishTransform(stamp, last_good_pose_);
      }
      return;
    }

    geometry_msgs::msg::Pose pose_in_base;
    geometry_msgs::msg::Twist twist_in_base;
    if (!transformPoseToBase(*msg, pose_in_base, twist_in_base)) {
      if (publish_tf_ && has_last_good_pose_) {
        const rclcpp::Time stamp =
            use_host_time_ ? now() : rclcpp::Time(msg->header.stamp);
        publishTransform(stamp, last_good_pose_);
      }
      return;
    }

    if (!isValidPose(pose_in_base)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping invalid rebased odometry; holding last good odom/TF");
      if (publish_tf_ && has_last_good_pose_) {
        const rclcpp::Time stamp =
            use_host_time_ ? now() : rclcpp::Time(msg->header.stamp);
        publishTransform(stamp, last_good_pose_);
      }
      return;
    }

    if (!has_z_offset_) {
      z_offset_ = seed_z_ - pose_in_base.position.z;
      has_z_offset_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Initialized odom z offset: seed_z=%.3f m raw_z=%.3f m offset=%.3f m",
          seed_z_, pose_in_base.position.z, z_offset_);
    }

    nav_msgs::msg::Odometry corrected = *msg;
    corrected.header.frame_id = odom_frame_;
    corrected.child_frame_id = base_frame_;
    corrected.pose.pose = pose_in_base;
    corrected.pose.pose.position.z += z_offset_;
    corrected.pose.pose.orientation =
        normalizedQuaternion(corrected.pose.pose.orientation);
    corrected.twist.twist = twist_in_base;

    if (use_host_time_) {
      corrected.header.stamp = now();
    }

    output_odom_pub_->publish(corrected);

    if (publish_tf_) {
      publishTransform(corrected.header.stamp, corrected.pose.pose);
    }
  }

  std::string raw_odom_topic_;
  std::string output_odom_topic_;
  std::string seed_pose_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  bool use_host_time_{false};
  bool transform_input_to_base_{false};
  double tf_lookup_timeout_sec_{0.2};

  bool has_seed_pose_{false};
  bool has_z_offset_{false};
  bool has_last_good_pose_{false};
  double seed_z_{0.0};
  double z_offset_{0.0};
  geometry_msgs::msg::Pose seed_pose_;
  geometry_msgs::msg::Pose last_good_pose_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr seed_pose_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRebaseNode>());
  rclcpp::shutdown();
  return 0;
}
