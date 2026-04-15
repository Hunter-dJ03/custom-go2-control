// Copyright 2025 go2_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
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
    declare_parameter<std::string>("frame_id", "front_camera_link");

    multicast_addr_ = get_parameter("multicast_address").as_string();
    multicast_port_ = get_parameter("multicast_port").as_int();
    iface_ = get_parameter("network_interface").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    target_fps_ = get_parameter("target_fps").as_int();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    frame_id_ = get_parameter("frame_id").as_string();

    auto qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "~/image_raw/compressed", qos);

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
