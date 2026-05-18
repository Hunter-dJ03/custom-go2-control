// Cartesian: p' = scale * p + offset in the incoming cloud axes (msg.header.frame_id).
// Spherical: same ray direction from the cloud origin; r' = max(0, range_scale * r + range_offset_m).
// Static TF does not change point coordinates. If use_input_frame_id is true, output keeps frame_id.
// min_range_m > 0 drops returns whose raw range r = ||p|| is below the threshold (sensor frame).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
/// Below this range (m), direction is ill-defined; leave (x,y,z) unchanged.
constexpr double kRangeEpsilon = 1e-9;

bool calibration_mode_is_spherical(const std::string & mode)
{
  return mode == "spherical";
}

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

bool field_offset(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name, uint32_t * out_offset)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name && f.count == 1U) {
      *out_offset = f.offset;
      return true;
    }
  }
  return false;
}

}  // namespace

class LidarCalibrationNode : public rclcpp::Node
{
public:
  LidarCalibrationNode()
  : Node("lidar_calibration_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/utlidar/cloud_deskewed");
    output_topic_ = declare_parameter<std::string>("output_topic", "/go2/lidar/points_calibrated");
    const std::string calibration_mode =
      declare_parameter<std::string>("calibration_mode", "cartesian");
    use_spherical_range_.store(calibration_mode_is_spherical(calibration_mode));
    scale_.store(declare_parameter<double>("scale", 1.0));
    offset_x_.store(declare_parameter<double>("offset_x", 0.0));
    offset_y_.store(declare_parameter<double>("offset_y", 0.0));
    offset_z_.store(declare_parameter<double>("offset_z", 0.0));
    range_scale_.store(declare_parameter<double>("range_scale", 1.0));
    range_offset_m_.store(declare_parameter<double>("range_offset_m", 0.0));
    min_range_m_.store(declare_parameter<double>("min_range_m", 0.0));
    declare_parameter<bool>("broadcast_radar_tf", false);
    declare_parameter<std::string>("tf_parent_frame", "radar");
    declare_parameter<std::string>("tf_child_frame", "utlidar_lidar");
    declare_parameter<double>("tf_yaw_deg", 0.0);
    declare_parameter<double>("tf_pitch_deg", 0.0);
    declare_parameter<double>("tf_roll_deg", 0.0);
    declare_parameter<double>("tf_x_m", 0.0);
    declare_parameter<double>("tf_y_m", 0.0);
    declare_parameter<double>("tf_z_m", 0.0);
    declare_parameter<bool>("use_input_frame_id", true);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "odom");
    use_input_frame_id_.store(get_parameter("use_input_frame_id").as_bool());
    restamp_.store(declare_parameter<bool>("restamp", true));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarCalibrationNode::onCloud, this, std::placeholders::_1));

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&LidarCalibrationNode::onParam, this, std::placeholders::_1));

    publishStaticRadarTf();

    if (use_spherical_range_.load()) {
      RCLCPP_INFO(
        get_logger(),
        "Subscribing \"%s\", publishing \"%s\"; calibration_mode=spherical "
        "(use_input_frame_id=%s). range_scale=%.6f range_offset_m=%.6f min_range_m=%.3f",
        input_topic_.c_str(), output_topic_.c_str(),
        use_input_frame_id_.load() ? "true" : "false",
        range_scale_.load(), range_offset_m_.load(), min_range_m_.load());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Subscribing \"%s\", publishing \"%s\"; calibration_mode=cartesian "
        "(use_input_frame_id=%s). scale=%.6f offset=(%.6f, %.6f, %.6f) min_range_m=%.3f",
        input_topic_.c_str(), output_topic_.c_str(),
        use_input_frame_id_.load() ? "true" : "false",
        scale_.load(), offset_x_.load(), offset_y_.load(), offset_z_.load(),
        min_range_m_.load());
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool resend_static_tf = false;
    for (const rclcpp::Parameter & p : parameters) {
      if (
        p.get_name() == "calibration_mode" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        use_spherical_range_.store(calibration_mode_is_spherical(p.as_string()));
      } else if (p.get_name() == "scale" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        scale_.store(p.as_double());
      } else if (p.get_name() == "offset_x" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        offset_x_.store(p.as_double());
      } else if (p.get_name() == "offset_y" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        offset_y_.store(p.as_double());
      } else if (p.get_name() == "offset_z" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        offset_z_.store(p.as_double());
      } else if (
        p.get_name() == "range_scale" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        range_scale_.store(p.as_double());
      } else if (
        p.get_name() == "range_offset_m" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        range_offset_m_.store(p.as_double());
      } else if (
        p.get_name() == "min_range_m" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        min_range_m_.store(p.as_double());
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
      } else if (
        p.get_name() == "restamp" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        restamp_.store(p.as_bool());
      } else if (p.get_name() == "tf_yaw_deg" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_pitch_deg" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_roll_deg" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_x_m" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_y_m" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        resend_static_tf = true;
      } else if (
        p.get_name() == "tf_z_m" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
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
    const double pitch_deg = get_parameter("tf_pitch_deg").as_double();
    const double roll_deg = get_parameter("tf_roll_deg").as_double();
    const double tx = get_parameter("tf_x_m").as_double();
    const double ty = get_parameter("tf_y_m").as_double();
    const double tz = get_parameter("tf_z_m").as_double();

    // Intrinsic ZYX (yaw about Z, then pitch about new Y, then roll about new X) — matches
    // tf2::Quaternion::setRPY and the deprecated static_transform_publisher "yaw pitch roll" order.
    const double cy = std::cos(0.5 * yaw_deg * kDegToRad);
    const double sy = std::sin(0.5 * yaw_deg * kDegToRad);
    const double cp = std::cos(0.5 * pitch_deg * kDegToRad);
    const double sp = std::sin(0.5 * pitch_deg * kDegToRad);
    const double cr = std::cos(0.5 * roll_deg * kDegToRad);
    const double sr = std::sin(0.5 * roll_deg * kDegToRad);
    const double qw = cr * cp * cy + sr * sp * sy;
    const double qx = sr * cp * cy - cr * sp * sy;
    const double qy = cr * sp * cy + sr * cp * sy;
    const double qz = cr * cp * sy - sr * sp * cy;

    if (!tf_static_) {
      tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.translation.z = tz;
    t.transform.rotation.x = qx;
    t.transform.rotation.y = qy;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    tf_static_->sendTransform(t);
    RCLCPP_INFO(
      get_logger(),
      "Static TF \"%s\" -> \"%s\": xyz = (%.3f, %.3f, %.3f) m, rpy = (%.3f, %.3f, %.3f) deg",
      parent.c_str(), child.c_str(), tx, ty, tz, roll_deg, pitch_deg, yaw_deg);
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
    if (
      dtype != sensor_msgs::msg::PointField::FLOAT32 &&
      dtype != sensor_msgs::msg::PointField::FLOAT64)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported x/y/z PointField datatype %u; need FLOAT32 or FLOAT64", dtype);
      return;
    }

    const bool spherical = use_spherical_range_.load();
    const double s = scale_.load();
    const double ox = offset_x_.load();
    const double oy = offset_y_.load();
    const double oz = offset_z_.load();
    const double rs = range_scale_.load();
    const double roff = range_offset_m_.load();
    const double min_r = min_range_m_.load();
    const bool filter = min_r > 0.0;

    if (!filter) {
      sensor_msgs::msg::PointCloud2 out = *msg;
      const size_t n = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
      applyInPlace(out, n, dtype, spherical, s, ox, oy, oz, rs, roff);
      finalizeAndPublish(out);
      return;
    }

    uint32_t off_x = 0, off_y = 0, off_z = 0;
    if (!field_offset(*msg, "x", &off_x) || !field_offset(*msg, "y", &off_y) ||
        !field_offset(*msg, "z", &off_z))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 missing x/y/z field offsets");
      return;
    }

    const size_t point_step = msg->point_step;
    const size_t n_in = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.fields = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step = msg->point_step;
    out.height = 1;
    out.data.resize(n_in * point_step);

    size_t kept = 0;
    const uint8_t * in_ptr = msg->data.data();
    uint8_t * out_ptr = out.data.data();
    for (size_t i = 0; i < n_in; ++i) {
      const uint8_t * row = in_ptr + i * point_step;
      double xd = 0.0, yd = 0.0, zd = 0.0;
      if (!readXyz(row, off_x, off_y, off_z, dtype, &xd, &yd, &zd)) {
        continue;
      }
      const double r = std::sqrt(xd * xd + yd * yd + zd * zd);
      if (r < min_r) {
        continue;
      }
      double xo = xd, yo = yd, zo = zd;
      transformPoint(spherical, s, ox, oy, oz, rs, roff, r, &xo, &yo, &zo);
      uint8_t * dst = out_ptr + kept * point_step;
      std::memcpy(dst, row, point_step);
      writeXyz(dst, off_x, off_y, off_z, dtype, xo, yo, zo);
      ++kept;
    }

    out.width = static_cast<uint32_t>(kept);
    out.row_step = static_cast<uint32_t>(point_step * kept);
    out.is_dense = true;
    out.data.resize(out.row_step);
    finalizeAndPublish(out);
  }

  void applyInPlace(
    sensor_msgs::msg::PointCloud2 & out, size_t n, uint8_t dtype, bool spherical, double s,
    double ox, double oy, double oz, double rs, double roff) const
  {
    if (dtype == sensor_msgs::msg::PointField::FLOAT32) {
      sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
      sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
      sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
      for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const float x = *ix, y = *iy, z = *iz;
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
          continue;
        }
        const double xd = x, yd = y, zd = z;
        const double r = std::sqrt(xd * xd + yd * yd + zd * zd);
        double xo = xd, yo = yd, zo = zd;
        transformPoint(spherical, s, ox, oy, oz, rs, roff, r, &xo, &yo, &zo);
        *ix = static_cast<float>(xo);
        *iy = static_cast<float>(yo);
        *iz = static_cast<float>(zo);
      }
    } else {
      sensor_msgs::PointCloud2Iterator<double> ix(out, "x");
      sensor_msgs::PointCloud2Iterator<double> iy(out, "y");
      sensor_msgs::PointCloud2Iterator<double> iz(out, "z");
      for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const double x = *ix, y = *iy, z = *iz;
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
          continue;
        }
        const double r = std::sqrt(x * x + y * y + z * z);
        double xo = x, yo = y, zo = z;
        transformPoint(spherical, s, ox, oy, oz, rs, roff, r, &xo, &yo, &zo);
        *ix = xo;
        *iy = yo;
        *iz = zo;
      }
    }
  }

  static void transformPoint(
    bool spherical, double s, double ox, double oy, double oz, double rs, double roff, double r,
    double * x, double * y, double * z)
  {
    if (spherical) {
      if (r < kRangeEpsilon) {
        return;
      }
      double r_new = rs * r + roff;
      if (r_new < 0.0) {
        r_new = 0.0;
      }
      const double k = r_new / r;
      *x = k * (*x);
      *y = k * (*y);
      *z = k * (*z);
    } else {
      *x = s * (*x) + ox;
      *y = s * (*y) + oy;
      *z = s * (*z) + oz;
    }
  }

  static bool readXyz(
    const uint8_t * row, uint32_t off_x, uint32_t off_y, uint32_t off_z, uint8_t dtype,
    double * x, double * y, double * z)
  {
    if (dtype == sensor_msgs::msg::PointField::FLOAT32) {
      float fx, fy, fz;
      std::memcpy(&fx, row + off_x, sizeof(float));
      std::memcpy(&fy, row + off_y, sizeof(float));
      std::memcpy(&fz, row + off_z, sizeof(float));
      if (std::isnan(fx) || std::isnan(fy) || std::isnan(fz)) {
        return false;
      }
      *x = fx;
      *y = fy;
      *z = fz;
      return true;
    }
    double dx, dy, dz;
    std::memcpy(&dx, row + off_x, sizeof(double));
    std::memcpy(&dy, row + off_y, sizeof(double));
    std::memcpy(&dz, row + off_z, sizeof(double));
    if (std::isnan(dx) || std::isnan(dy) || std::isnan(dz)) {
      return false;
    }
    *x = dx;
    *y = dy;
    *z = dz;
    return true;
  }

  static void writeXyz(
    uint8_t * row, uint32_t off_x, uint32_t off_y, uint32_t off_z, uint8_t dtype, double x,
    double y, double z)
  {
    if (dtype == sensor_msgs::msg::PointField::FLOAT32) {
      const float fx = static_cast<float>(x);
      const float fy = static_cast<float>(y);
      const float fz = static_cast<float>(z);
      std::memcpy(row + off_x, &fx, sizeof(float));
      std::memcpy(row + off_y, &fy, sizeof(float));
      std::memcpy(row + off_z, &fz, sizeof(float));
      return;
    }
    std::memcpy(row + off_x, &x, sizeof(double));
    std::memcpy(row + off_y, &y, sizeof(double));
    std::memcpy(row + off_z, &z, sizeof(double));
  }

  void finalizeAndPublish(sensor_msgs::msg::PointCloud2 & out)
  {
    if (!use_input_frame_id_.load()) {
      std::lock_guard<std::mutex> lock(output_frame_mutex_);
      out.header.frame_id = output_frame_id_;
    }
    // Unitree's /utlidar/cloud ships stamps that lag the Orin's ROS clock by minutes; that
    // breaks any dynamic TF lookup (odom/map) downstream. Re-stamp with now() so RViz / Nav2
    // can resolve TF against the current cache. Disable via `restamp:=false` if you need to
    // preserve the upstream sensor time (e.g. for offline alignment with other sensors).
    if (restamp_.load()) {
      out.header.stamp = now();
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
  std::atomic<double> range_scale_;
  std::atomic<double> range_offset_m_;
  std::atomic<double> min_range_m_{0.0};
  std::atomic<bool> use_spherical_range_{false};
  std::atomic<bool> use_input_frame_id_{true};
  std::atomic<bool> restamp_{true};

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
