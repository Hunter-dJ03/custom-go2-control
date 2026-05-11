// Scale/offset apply to (x,y,z) in the incoming cloud's axes — i.e. the frame named in
// msg.header.frame_id from the driver (gathered frame). Static TF does not change those numbers.
// If use_input_frame_id is true (default), the published cloud keeps that same frame_id.

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool field_datatype(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name,
  uint8_t * out_datatype)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name && f.count == 1U) {
      *out_datatype = f.datatype;
      return true;
    }
  }
  return false;
}

bool has_xyz_same_type(const sensor_msgs::msg::PointCloud2 & cloud, uint8_t * dtype)
{
  uint8_t dx = 0, dy = 0, dz = 0;
  if (!field_datatype(cloud, "x", &dx)) {
    return false;
  }
  if (!field_datatype(cloud, "y", &dy) || !field_datatype(cloud, "z", &dz)) {
    return false;
  }
  if (dx != dy || dy != dz) {
    return false;
  }
  *dtype = dx;
  return true;
}

}  // namespace

class LidarCalibrationNode : public rclcpp::Node
{
public:
  LidarCalibrationNode()
  : Node("lidar_calibration_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/utlidar/cloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/go2/lidar/points_calibrated");
    scale_.store(declare_parameter<double>("scale", 1.0));
    offset_x_.store(declare_parameter<double>("offset_x", 0.0));
    offset_y_.store(declare_parameter<double>("offset_y", 0.0));
    offset_z_.store(declare_parameter<double>("offset_z", 0.0));
    declare_parameter<bool>("broadcast_radar_tf", true);
    declare_parameter<std::string>("tf_parent_frame", "radar");
    declare_parameter<std::string>("tf_child_frame", "utlidar_lidar");
    declare_parameter<double>("tf_yaw_deg", 120.0);
    declare_parameter<bool>("use_input_frame_id", true);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "utlidar_lidar");
    use_input_frame_id_.store(get_parameter("use_input_frame_id").as_bool());

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarCalibrationNode::onCloud, this, std::placeholders::_1));

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&LidarCalibrationNode::onParam, this, std::placeholders::_1));

    publishStaticRadarTf();

    RCLCPP_INFO(
      get_logger(),
      "Subscribing \"%s\", publishing \"%s\"; scale/offset in each message's input frame_id "
      "(use_input_frame_id=%s). scale=%.6f offset=(%.6f, %.6f, %.6f)",
      input_topic_.c_str(), output_topic_.c_str(),
      use_input_frame_id_.load() ? "true" : "false",
      scale_.load(), offset_x_.load(), offset_y_.load(), offset_z_.load());
  }

private:
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool resend_static_tf = false;
    for (const rclcpp::Parameter & p : parameters) {
      if (p.get_name() == "scale" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        scale_.store(p.as_double());
      } else if (p.get_name() == "offset_x" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        offset_x_.store(p.as_double());
      } else if (p.get_name() == "offset_y" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        offset_y_.store(p.as_double());
      } else if (p.get_name() == "offset_z" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        offset_z_.store(p.as_double());
      } else if (
        p.get_name() == "output_frame_id" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        std::lock_guard<std::mutex> lock(output_frame_mutex_);
        output_frame_id_ = p.as_string();
      } else if (
        p.get_name() == "use_input_frame_id" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        use_input_frame_id_.store(p.as_bool());
      } else if (p.get_name() == "tf_yaw_deg" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_parent_frame" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_child_frame" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        resend_static_tf = true;
      } else if (
        p.get_name() == "broadcast_radar_tf" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        resend_static_tf = true;
      }
    }
    if (resend_static_tf) {
      publishStaticRadarTf();
    }
    return result;
  }

  void publishStaticRadarTf()
  {
    if (!get_parameter("broadcast_radar_tf").as_bool()) {
      return;
    }
    const std::string parent = get_parameter("tf_parent_frame").as_string();
    const std::string child = get_parameter("tf_child_frame").as_string();
    const double yaw_deg = get_parameter("tf_yaw_deg").as_double();
    const double yaw_rad = yaw_deg * kDegToRad;
    const double half = 0.5 * yaw_rad;

    if (!tf_static_) {
      tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(half);
    t.transform.rotation.w = std::cos(half);
    tf_static_->sendTransform(t);
    RCLCPP_INFO(
      get_logger(),
      "Static TF \"%s\" -> \"%s\": yaw_z = %.3f deg (parent frame)",
      parent.c_str(), child.c_str(), yaw_deg);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    uint8_t dtype = 0;
    if (!has_xyz_same_type(*msg, &dtype)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 missing x/y/z or mixed dtypes; skipping");
      return;
    }

    sensor_msgs::msg::PointCloud2 out = *msg;
    const size_t n = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);

    const double s = scale_.load();
    const double ox = offset_x_.load();
    const double oy = offset_y_.load();
    const double oz = offset_z_.load();

    if (dtype == sensor_msgs::msg::PointField::FLOAT32) {
      sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
      sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
      sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
      for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const float x = *ix;
        const float y = *iy;
        const float z = *iz;
        if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
          *ix = static_cast<float>(s * static_cast<double>(x) + ox);
          *iy = static_cast<float>(s * static_cast<double>(y) + oy);
          *iz = static_cast<float>(s * static_cast<double>(z) + oz);
        }
      }
    } else if (dtype == sensor_msgs::msg::PointField::FLOAT64) {
      sensor_msgs::PointCloud2Iterator<double> ix(out, "x");
      sensor_msgs::PointCloud2Iterator<double> iy(out, "y");
      sensor_msgs::PointCloud2Iterator<double> iz(out, "z");
      for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const double x = *ix;
        const double y = *iy;
        const double z = *iz;
        if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
          *ix = s * x + ox;
          *iy = s * y + oy;
          *iz = s * z + oz;
        }
      }
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported x/y/z PointField datatype %u; need FLOAT32 or FLOAT64", dtype);
      return;
    }

    if (!use_input_frame_id_.load()) {
      std::lock_guard<std::mutex> lock(output_frame_mutex_);
      out.header.frame_id = output_frame_id_;
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::mutex output_frame_mutex_;
  std::atomic<double> scale_;
  std::atomic<double> offset_x_;
  std::atomic<double> offset_y_;
  std::atomic<double> offset_z_;
  std::atomic<bool> use_input_frame_id_{true};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
