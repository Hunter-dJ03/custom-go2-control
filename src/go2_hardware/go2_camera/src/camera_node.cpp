// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <yaml-cpp/yaml.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraNode : public rclcpp::Node {
 public:
  CameraNode() : Node("go2_camera_node") {
    declare_parameter<std::string>("multicast_address", "230.1.1.1");
    declare_parameter<int>("multicast_port", 1720);
    declare_parameter<std::string>("network_interface", "enp2s0");
    declare_parameter<int>("width", 1280);
    declare_parameter<int>("height", 720);
    declare_parameter<int>("target_fps", 30);
    declare_parameter<int>("jpeg_quality", 80);
    declare_parameter<std::string>("frame_id", "front_camera");
    declare_parameter<std::string>("calibration_file", "");

    multicast_addr_ = get_parameter("multicast_address").as_string();
    multicast_port_ = get_parameter("multicast_port").as_int();
    iface_ = get_parameter("network_interface").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    target_fps_ = get_parameter("target_fps").as_int();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    calibration_file_ = get_parameter("calibration_file").as_string();

    auto qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "~/image_raw/compressed", qos);
    camera_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);

    if (!loadCalibrationForHeight(static_cast<int>(height_))) {
      RCLCPP_WARN(get_logger(),
                  "Camera calibration not loaded; ~/camera_info will not be published "
                  "until a valid calibration YAML is found (see calibration_file param).");
    }

    auto period = std::chrono::duration<double>(1.0 / target_fps_);
    timer_ = create_wall_timer(period, std::bind(&CameraNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "Go2 camera node starting — multicast %s:%ld on %s, target %ld fps",
                multicast_addr_.c_str(), multicast_port_, iface_.c_str(), target_fps_);
  }

  ~CameraNode() override { closePipeline(); }

 private:
  bool openPipeline() {
    std::string pipeline_str =
        "udpsrc address=" + multicast_addr_ +
        " port=" + std::to_string(multicast_port_) +
        " multicast-iface=" + iface_ +
        " ! application/x-rtp,media=video,encoding-name=H264"
        " ! rtph264depay ! h264parse ! avdec_h264"
        " ! videoconvert ! video/x-raw,format=BGR"
        " ! appsink name=sink drop=true max-buffers=1";

    RCLCPP_INFO(get_logger(), "Opening GStreamer pipeline: %s", pipeline_str.c_str());

    GError *err = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
    if (err) {
      RCLCPP_ERROR(get_logger(), "Failed to parse pipeline: %s", err->message);
      g_error_free(err);
      return false;
    }

    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!sink) {
      RCLCPP_ERROR(get_logger(), "Could not find appsink element");
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      return false;
    }
    appsink_ = GST_APP_SINK(sink);

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(),
                   "Pipeline failed to reach PLAYING state. "
                   "Check that interface '%s' is up and receiving "
                   "multicast traffic from the Go2.",
                   iface_.c_str());
      closePipeline();
      return false;
    }

    pipeline_ok_ = true;
    RCLCPP_INFO(get_logger(), "GStreamer pipeline opened successfully");
    return true;
  }

  void closePipeline() {
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      if (appsink_) {
        gst_object_unref(appsink_);
        appsink_ = nullptr;
      }
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      pipeline_ok_ = false;
      RCLCPP_INFO(get_logger(), "GStreamer pipeline stopped");
    }
  }

  void tick() {
    if (!pipeline_ok_) {
      if (!openPipeline()) return;
    }

    GstSample *sample = gst_app_sink_try_pull_sample(appsink_, 0);
    if (!sample) return;

    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);

    int w = width_;
    int h = height_;
    if (caps) {
      GstStructure *s = gst_caps_get_structure(caps, 0);
      gst_structure_get_int(s, "width", &w);
      gst_structure_get_int(s, "height", &h);
    }

    ensureCalibrationMatchesStream(w, h);

    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return;
    }

    cv::Mat frame(h, w, CV_8UC3, map.data);
    auto stamp = now();

    bool has_raw = image_pub_->get_subscription_count() > 0;
    bool has_comp = compressed_pub_->get_subscription_count() > 0;

    if (has_raw) publishRaw(frame, stamp);
    if (has_comp) publishCompressed(frame, stamp);
    if (calibration_loaded_ && (has_raw || has_comp)) publishCameraInfo(stamp);

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);

    if (has_raw || has_comp) updateFpsStats();
  }

  void publishRaw(const cv::Mat &frame, const rclcpp::Time &stamp) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id_;
    image_pub_->publish(*msg);
  }

  void publishCompressed(const cv::Mat &frame, const rclcpp::Time &stamp) {
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, static_cast<int>(jpeg_quality_)};
    std::vector<uchar> jpeg_buf;
    if (!cv::imencode(".jpg", frame, jpeg_buf, params)) return;

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data.assign(jpeg_buf.begin(), jpeg_buf.end());
    compressed_pub_->publish(msg);
  }

  void publishCameraInfo(const rclcpp::Time &stamp) {
    auto msg = camera_info_;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    camera_info_pub_->publish(msg);
  }

  std::string resolveCalibrationPath(int image_height) {
    if (!calibration_file_.empty()) return calibration_file_;
    try {
      std::string share = ament_index_cpp::get_package_share_directory("go2_camera");
      return share + "/calibration/front_camera_" + std::to_string(image_height) + ".yaml";
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Could not resolve package share for go2_camera: %s",
                   e.what());
      return {};
    }
  }

  bool loadCameraInfoYaml(const std::string &path, sensor_msgs::msg::CameraInfo *out) {
    YAML::Node root = YAML::LoadFile(path);
    out->width = root["image_width"].as<uint32_t>();
    out->height = root["image_height"].as<uint32_t>();
    out->distortion_model = root["distortion_model"].as<std::string>();

    const YAML::Node &k = root["camera_matrix"]["data"];
    for (size_t i = 0; i < 9; ++i) {
      out->k[i] = k[i].as<double>();
    }

    out->d.clear();
    const YAML::Node &d = root["distortion_coefficients"]["data"];
    for (size_t i = 0; i < d.size(); ++i) {
      out->d.push_back(d[i].as<double>());
    }

    const YAML::Node &r = root["rectification_matrix"]["data"];
    for (size_t i = 0; i < 9; ++i) {
      out->r[i] = r[i].as<double>();
    }

    const YAML::Node &p = root["projection_matrix"]["data"];
    for (size_t i = 0; i < 12; ++i) {
      out->p[i] = p[i].as<double>();
    }

    return true;
  }

  void ensureCalibrationMatchesStream(int w, int h) {
    if (calibration_loaded_ && static_cast<uint32_t>(w) == camera_info_.width &&
        static_cast<uint32_t>(h) == camera_info_.height) {
      return;
    }
    if (!loadCalibrationForHeight(h)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No camera calibration for stream size %dx%d (expected "
          "share/go2_camera/calibration/front_camera_%d.yaml or calibration_file).",
          w, h, h);
    }
  }

  bool loadCalibrationForHeight(int image_height) {
    std::string path = resolveCalibrationPath(image_height);
    if (path.empty()) return false;
    try {
      loadCameraInfoYaml(path, &camera_info_);
      calibration_loaded_ = true;
      RCLCPP_INFO(get_logger(), "Loaded camera calibration from %s", path.c_str());
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load camera calibration '%s': %s",
                   path.c_str(), e.what());
      calibration_loaded_ = false;
      return false;
    }
  }

  void updateFpsStats() {
    ++frames_published_;
    auto now_mono = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now_mono - fps_report_time_).count();
    if (elapsed >= 5.0) {
      double fps = frames_published_ / elapsed;
      RCLCPP_INFO(get_logger(), "Camera publishing at %.1f fps", fps);
      frames_published_ = 0;
      fps_report_time_ = now_mono;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  GstElement *pipeline_ = nullptr;
  GstAppSink *appsink_ = nullptr;
  bool pipeline_ok_ = false;

  std::string multicast_addr_;
  int64_t multicast_port_;
  std::string iface_;
  int64_t width_;
  int64_t height_;
  int64_t target_fps_;
  int64_t jpeg_quality_;
  std::string frame_id_;
  std::string calibration_file_;

  sensor_msgs::msg::CameraInfo camera_info_;
  bool calibration_loaded_ = false;

  uint32_t frames_published_ = 0;
  std::chrono::steady_clock::time_point fps_report_time_ =
      std::chrono::steady_clock::now();
};

int main(int argc, char **argv) {
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
