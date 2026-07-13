#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "MapPoint.h"
#include "System.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hanmole_sdk/api/sdk.hpp"
#include "hanmole_sdk/data/frame.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr int kTrackingOk = 2;
constexpr int kTrackingRecentlyLost = 3;
constexpr int kTrackingOkKlt = 5;
constexpr int kMaxDirectImageQueueSize = 4;
constexpr int kMaxDirectPairQueueSize = 2;

enum class SlamSensorMode
{
  kStereo,
  kImuStereo,
};

enum class ImageInputMode
{
  kRosTopics,
  kSdkDirect,
};

struct DirectImageFrame
{
  std::vector<std::uint8_t> image_payload;
  std::uint16_t width{0};
  std::uint16_t height{0};
  std::uint32_t stride_bytes{0};
  std::uint32_t pixel_format{0};
  builtin_interfaces::msg::Time stamp;
  std::uint64_t capture_timestamp_ns{0};
  std::uint64_t seq{0};
};

struct DirectImagePair
{
  DirectImageFrame left;
  DirectImageFrame right;
};

#pragma pack(push, 1)
struct CameraPayloadHeader
{
  std::uint32_t magic{0};
  std::uint16_t version{0};
  std::uint16_t flags{0};
  std::uint64_t timestamp_ns{0};
  std::uint16_t width{0};
  std::uint16_t height{0};
  std::uint32_t stride_bytes{0};
  std::uint32_t pixel_format{0};
  std::uint32_t fps{0};
  std::uint32_t image_size{0};
};
#pragma pack(pop)

constexpr std::uint32_t kCameraPayloadMagic = 0x314D4348U;
constexpr std::uint32_t kCameraPixelMjpeg = 1U;
constexpr std::uint32_t kCameraPixelRgb8 = 2U;
constexpr std::uint32_t kCameraPixelBgr8 = 3U;
constexpr std::uint32_t kCameraPixelGray8 = 4U;

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool is_finite(double value)
{
  return std::isfinite(value);
}

bool scalar_is_number(const std::string & value)
{
  if (value.empty()) {
    return false;
  }

  char * end = nullptr;
  std::strtod(value.c_str(), &end);
  return end != value.c_str() && *end == '\0';
}

SlamSensorMode parse_sensor_mode(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      if (ch == '-') {
        return '_';
      }
      return static_cast<char>(std::tolower(ch));
    });

  if (value == "stereo" || value == "vo") {
    return SlamSensorMode::kStereo;
  }
  if (value == "imu_stereo" || value == "stereo_inertial" || value == "vio") {
    return SlamSensorMode::kImuStereo;
  }
  throw std::runtime_error(
          "orbslam3.sensor_mode must be one of: stereo, imu_stereo; got: " + value);
}

const char * sensor_mode_name(SlamSensorMode mode)
{
  switch (mode) {
    case SlamSensorMode::kStereo:
      return "stereo";
    case SlamSensorMode::kImuStereo:
      return "imu_stereo";
  }
  return "unknown";
}

ImageInputMode parse_image_input_mode(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      if (ch == '-') {
        return '_';
      }
      return static_cast<char>(std::tolower(ch));
    });

  if (value == "ros" || value == "ros_topics" || value == "topic") {
    return ImageInputMode::kRosTopics;
  }
  if (value == "sdk" || value == "sdk_direct" || value == "shm" || value == "sdk_shm") {
    return ImageInputMode::kSdkDirect;
  }
  throw std::runtime_error(
          "ros__parameters.image_input_mode must be one of: sdk_direct, ros_topics; got: " + value);
}

const char * image_input_mode_name(ImageInputMode mode)
{
  switch (mode) {
    case ImageInputMode::kRosTopics:
      return "ros_topics";
    case ImageInputMode::kSdkDirect:
      return "sdk_direct";
  }
  return "unknown";
}

std::string quote_opencv_string(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') {
      stream << '\\';
    }
    stream << ch;
  }
  stream << '"';
  return stream.str();
}

template<typename T>
T yaml_value_or(const YAML::Node & node, const std::string & key, const T & fallback)
{
  const YAML::Node value = node ? node[key] : YAML::Node();
  if (!value) {
    return fallback;
  }
  return value.as<T>();
}

YAML::Node require_map(const YAML::Node & node, const std::string & key)
{
  const YAML::Node child = node[key];
  if (!child || !child.IsMap()) {
    throw std::runtime_error("vio config requires map section: " + key);
  }
  return child;
}

Eigen::Matrix4f read_matrix4(const YAML::Node & node, const std::string & key)
{
  const YAML::Node matrix = node[key];
  if (!matrix || !matrix.IsMap()) {
    throw std::runtime_error("vio config requires matrix: " + key);
  }
  const int rows = yaml_value_or<int>(matrix, "rows", 0);
  const int cols = yaml_value_or<int>(matrix, "cols", 0);
  const YAML::Node data = matrix["data"];
  if (rows != 4 || cols != 4 || !data || !data.IsSequence() || data.size() != 16) {
    throw std::runtime_error(key + " must be a 4x4 matrix with 16 values");
  }

  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      result(row, col) = data[static_cast<std::size_t>(row * 4 + col)].as<float>();
    }
  }
  return result;
}

void write_opencv_matrix(std::ostream & stream, const std::string & key, const YAML::Node & matrix)
{
  const int rows = yaml_value_or<int>(matrix, "rows", 0);
  const int cols = yaml_value_or<int>(matrix, "cols", 0);
  const std::string dt = yaml_value_or<std::string>(matrix, "dt", "f");
  const YAML::Node data = matrix["data"];
  if (rows <= 0 || cols <= 0 || !data || !data.IsSequence()) {
    throw std::runtime_error("invalid OpenCV matrix in vio config: " + key);
  }

  stream << key << ": !!opencv-matrix\n";
  stream << "  rows: " << rows << "\n";
  stream << "  cols: " << cols << "\n";
  stream << "  dt: " << dt << "\n";
  stream << "  data: [";
  for (std::size_t index = 0; index < data.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << data[index].as<std::string>();
  }
  stream << "]\n";
}

void write_opencv_scalar(std::ostream & stream, const std::string & key, const YAML::Node & value)
{
  static const std::set<std::string> string_keys = {
    "Camera.type",
    "File.version",
    "System.LoadAtlasFromFile",
    "System.SaveAtlasToFile",
  };

  const std::string text = value.as<std::string>();
  stream << key << ": ";
  if (string_keys.find(key) != string_keys.end() || (!scalar_is_number(text) && text != "true" &&
    text != "false"))
  {
    stream << quote_opencv_string(text);
  } else {
    stream << text;
  }
  stream << "\n";
}

std::filesystem::path resolve_path_relative_to(
  const std::filesystem::path & base_file,
  const std::string & value)
{
  std::filesystem::path path(value);
  if (path.is_absolute()) {
    return path;
  }
  return base_file.parent_path() / path;
}

cv::Mat image_to_cv_mat(const sensor_msgs::msg::Image & message)
{
  const std::string & encoding = message.encoding;
  const int height = static_cast<int>(message.height);
  const int width = static_cast<int>(message.width);
  if (height <= 0 || width <= 0) {
    throw std::runtime_error("image has invalid dimensions");
  }

  auto make_mat = [&](int type, std::size_t channels) {
      const std::size_t min_step = static_cast<std::size_t>(width) * channels;
      if (message.step < min_step || message.data.size() < message.step * message.height) {
        throw std::runtime_error("image data is smaller than width/height/step require");
      }
      return cv::Mat(height, width, type, const_cast<unsigned char *>(message.data.data()),
        message.step);
    };

  if (encoding == "mono8" || encoding == "8UC1") {
    return make_mat(CV_8UC1, 1).clone();
  }
  if (encoding == "bgr8") {
    return make_mat(CV_8UC3, 3).clone();
  }
  if (encoding == "rgb8") {
    return make_mat(CV_8UC3, 3).clone();
  }
  if (encoding == "bgra8" || encoding == "rgba8") {
    return make_mat(CV_8UC4, 4).clone();
  }
  if (encoding == "mono16" || encoding == "16UC1") {
    cv::Mat source = make_mat(CV_16UC1, 2);
    cv::Mat converted;
    source.convertTo(converted, CV_8UC1, 1.0 / 256.0);
    return converted;
  }

  throw std::runtime_error("unsupported image encoding for ORB-SLAM3 VIO: " + encoding);
}

CameraPayloadHeader parse_camera_payload_header(const std::vector<std::byte> & payload)
{
  if (payload.size() < sizeof(CameraPayloadHeader)) {
    throw std::runtime_error("camera SDK payload too small");
  }
  CameraPayloadHeader header{};
  std::memcpy(&header, payload.data(), sizeof(header));
  if (header.magic != kCameraPayloadMagic || header.version != 1U) {
    throw std::runtime_error("unexpected camera SDK payload header");
  }
  if (sizeof(CameraPayloadHeader) + header.image_size > payload.size()) {
    throw std::runtime_error("camera SDK payload image overflow");
  }
  return header;
}

cv::Mat decode_direct_image_frame(const DirectImageFrame & frame)
{
  if (frame.image_payload.empty()) {
    throw std::runtime_error("camera SDK payload has empty image");
  }
  if (frame.image_payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::runtime_error("camera SDK payload image too large for OpenCV");
  }

  auto validate_raw_shape = [&frame](std::size_t bytes_per_pixel) {
      if (frame.width == 0U || frame.height == 0U) {
        throw std::runtime_error("camera SDK raw payload has invalid dimensions");
      }
      const std::size_t min_step = static_cast<std::size_t>(frame.width) * bytes_per_pixel;
      const std::size_t step = frame.stride_bytes == 0U ? min_step : frame.stride_bytes;
      if (step < min_step ||
        step > frame.image_payload.size() / static_cast<std::size_t>(frame.height))
      {
        throw std::runtime_error("camera SDK raw payload has invalid stride");
      }
      return step;
    };

  if (frame.pixel_format == kCameraPixelMjpeg) {
    cv::Mat encoded(
      1,
      static_cast<int>(frame.image_payload.size()),
      CV_8UC1,
      const_cast<std::uint8_t *>(frame.image_payload.data()));
    cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (decoded.empty()) {
      throw std::runtime_error("failed to decode camera SDK MJPEG payload");
    }
    return decoded;
  }

  if (frame.pixel_format == kCameraPixelBgr8 || frame.pixel_format == kCameraPixelRgb8) {
    const std::size_t step = validate_raw_shape(3U);
    cv::Mat source(
      static_cast<int>(frame.height),
      static_cast<int>(frame.width),
      CV_8UC3,
      const_cast<std::uint8_t *>(frame.image_payload.data()),
      step);
    cv::Mat gray;
    cv::cvtColor(
      source,
      gray,
      frame.pixel_format == kCameraPixelRgb8 ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
    return gray;
  }

  if (frame.pixel_format == kCameraPixelGray8) {
    const std::size_t step = validate_raw_shape(1U);
    return cv::Mat(
      static_cast<int>(frame.height),
      static_cast<int>(frame.width),
      CV_8UC1,
      const_cast<std::uint8_t *>(frame.image_payload.data()),
      step).clone();
  }

  throw std::runtime_error("unsupported camera SDK pixel format: " + std::to_string(frame.pixel_format));
}

geometry_msgs::msg::Pose eigen_pose_to_msg(const Eigen::Matrix4f & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);

  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  Eigen::Quaternionf quaternion(rotation);
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

geometry_msgs::msg::Pose eigen_pose_to_planar_msg(const Eigen::Matrix4f & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = 0.0;

  const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  const float yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::AngleAxisf yaw_rotation(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf quaternion(yaw_rotation);
  quaternion.normalize();
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

class Orbslam3StereoInertialNode : public rclcpp::Node
{
public:
  Orbslam3StereoInertialNode()
  : Node("orbslam3_stereo_inertial_node")
  {
    config_file_ = declare_parameter<std::string>("config_file", "");
    if (config_file_.empty()) {
      throw std::runtime_error("config_file parameter cannot be empty");
    }

    load_config();
    generate_orbslam3_settings();
    initialize_orbslam3();
    create_ros_interfaces();
    if (image_input_mode_ == ImageInputMode::kSdkDirect) {
      start_direct_sdk_capture();
    }

    if (sensor_mode_ == SlamSensorMode::kImuStereo) {
      RCLCPP_INFO(
        get_logger(),
        "ORB-SLAM3 stereo-inertial VIO input=%s left=%s right=%s imu=%s",
        image_input_mode_name(image_input_mode_),
        left_input_name().c_str(),
        right_input_name().c_str(),
        imu_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "ORB-SLAM3 stereo VO input=%s left=%s right=%s",
        image_input_mode_name(image_input_mode_),
        left_input_name().c_str(),
        right_input_name().c_str());
    }
  }

  ~Orbslam3StereoInertialNode() override
  {
    stop_direct_sdk_capture();
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    if (slam_) {
      slam_->Shutdown();
      slam_.reset();
    }
  }

private:
  void load_config()
  {
    const std::filesystem::path config_path(config_file_);
    YAML::Node root = YAML::LoadFile(config_file_);
    const YAML::Node orbslam3 = require_map(root, "orbslam3");
    const YAML::Node calibration = require_map(root, "camera_imu_calibration");
    const YAML::Node ros_parameters = require_map(root, "ros__parameters");

    calibration_ready_ = yaml_value_or<bool>(orbslam3, "calibration_ready", false);
    sensor_mode_ = parse_sensor_mode(
      yaml_value_or<std::string>(orbslam3, "sensor_mode", "imu_stereo"));
    vocabulary_file_ = resolve_path_relative_to(
      config_path,
      yaml_value_or<std::string>(
        orbslam3,
        "vocabulary",
        "/home/hanmole/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt")).string();
    library_root_ = resolve_path_relative_to(
      config_path,
      yaml_value_or<std::string>(
        orbslam3,
        "library_root",
        "/home/hanmole/third_party/ORB_SLAM3")).string();
    generated_settings_file_ = resolve_path_relative_to(
      config_path,
      yaml_value_or<std::string>(
        orbslam3,
        "settings_output",
        "/home/hanmole/ros2_ws/src/hanmole_navigation/.runtime/vio/orbslam3_settings.yaml")).string();
    use_viewer_ = yaml_value_or<bool>(orbslam3, "use_viewer", false);

    T_b_c1_ = read_matrix4(calibration, "IMU.T_b_c1");
    T_c1_b_ = T_b_c1_.inverse();
    T_base_c1_ = calibration["VO.T_base_c1"] ?
      read_matrix4(calibration, "VO.T_base_c1") : T_b_c1_;
    T_c1_base_ = T_base_c1_.inverse();
    T_nav_vio_ = calibration["VO.T_nav_vio"] ?
      read_matrix4(calibration, "VO.T_nav_vio") : Eigen::Matrix4f::Identity();
    calibration_section_ = calibration;

    left_image_topic_ = declare_parameter<std::string>(
      "left_image_topic",
      yaml_value_or<std::string>(
        ros_parameters,
        "left_image_topic",
        "/hanmole/sensor/camera_head_left/image_raw"));
    right_image_topic_ = declare_parameter<std::string>(
      "right_image_topic",
      yaml_value_or<std::string>(
        ros_parameters,
        "right_image_topic",
        "/hanmole/sensor/camera_head_right/image_raw"));
    const std::string image_input_mode_text = declare_parameter<std::string>(
      "image_input_mode",
      yaml_value_or<std::string>(ros_parameters, "image_input_mode", "sdk_direct"));
    image_input_mode_ = parse_image_input_mode(image_input_mode_text);
    sdk_config_ = declare_parameter<std::string>(
      "sdk_config",
      yaml_value_or<std::string>(ros_parameters, "sdk_config", "v0_1"));
    sdk_left_module_ = declare_parameter<std::string>(
      "sdk_left_module",
      yaml_value_or<std::string>(ros_parameters, "sdk_left_module", "mod_camera_head_left"));
    sdk_right_module_ = declare_parameter<std::string>(
      "sdk_right_module",
      yaml_value_or<std::string>(ros_parameters, "sdk_right_module", "mod_camera_head_right"));
    sdk_left_device_ = declare_parameter<std::string>(
      "sdk_left_device",
      yaml_value_or<std::string>(ros_parameters, "sdk_left_device", "camera_head_left"));
    sdk_right_device_ = declare_parameter<std::string>(
      "sdk_right_device",
      yaml_value_or<std::string>(ros_parameters, "sdk_right_device", "camera_head_right"));
    sdk_frame_timeout_ms_ = declare_parameter<int>(
      "sdk_frame_timeout_ms",
      yaml_value_or<int>(ros_parameters, "sdk_frame_timeout_ms", 1000));
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic",
      yaml_value_or<std::string>(ros_parameters, "imu_topic", "/imu/data"));
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic",
      yaml_value_or<std::string>(ros_parameters, "odom_topic", "/vio/odom"));
    publish_nav_odom_ = declare_parameter<bool>(
      "publish_nav_odom",
      yaml_value_or<bool>(ros_parameters, "publish_nav_odom", false));
    nav_odom_topic_ = declare_parameter<std::string>(
      "nav_odom_topic",
      yaml_value_or<std::string>(ros_parameters, "nav_odom_topic", "/vio/odom_2d"));
    nav_frame_id_ = declare_parameter<std::string>(
      "nav_frame_id",
      yaml_value_or<std::string>(ros_parameters, "nav_frame_id", "vio_odom"));
    nav_child_frame_id_ = declare_parameter<std::string>(
      "nav_child_frame_id",
      yaml_value_or<std::string>(ros_parameters, "nav_child_frame_id", "base_footprint"));
    project_nav_odom_2d_ = declare_parameter<bool>(
      "project_nav_odom_2d",
      yaml_value_or<bool>(ros_parameters, "project_nav_odom_2d", true));
    path_topic_ = declare_parameter<std::string>(
      "path_topic",
      yaml_value_or<std::string>(ros_parameters, "path_topic", "/vio/path"));
    camera_pose_topic_ = declare_parameter<std::string>(
      "camera_pose_topic",
      yaml_value_or<std::string>(ros_parameters, "camera_pose_topic", "/vio/camera_pose"));
    tracked_points_topic_ = declare_parameter<std::string>(
      "tracked_points_topic",
      yaml_value_or<std::string>(
        ros_parameters,
        "tracked_points_topic",
        "/vio/tracked_points"));
    publish_tracked_points_ = declare_parameter<bool>(
      "publish_tracked_points",
      yaml_value_or<bool>(ros_parameters, "publish_tracked_points", false));
    frame_id_ = declare_parameter<std::string>(
      "frame_id",
      yaml_value_or<std::string>(ros_parameters, "frame_id", "vio_odom"));
    child_frame_id_ = declare_parameter<std::string>(
      "child_frame_id",
      yaml_value_or<std::string>(ros_parameters, "child_frame_id", "base_link"));
    max_stereo_time_diff_sec_ = declare_parameter<double>(
      "max_stereo_time_diff_sec",
      yaml_value_or<double>(ros_parameters, "max_stereo_time_diff_sec", 0.01));
    max_image_queue_size_ = declare_parameter<int>(
      "max_image_queue_size",
      yaml_value_or<int>(ros_parameters, "max_image_queue_size", 10));
    max_imu_queue_size_ = declare_parameter<int>(
      "max_imu_queue_size",
      yaml_value_or<int>(ros_parameters, "max_imu_queue_size", 2000));
    max_imu_window_sec_ = declare_parameter<double>(
      "max_imu_window_sec",
      yaml_value_or<double>(ros_parameters, "max_imu_window_sec", 0.25));
    max_path_size_ = declare_parameter<int>(
      "max_path_size",
      yaml_value_or<int>(ros_parameters, "max_path_size", 2000));
    publish_tf_ = declare_parameter<bool>(
      "publish_tf",
      yaml_value_or<bool>(ros_parameters, "publish_tf", false));

    if (max_stereo_time_diff_sec_ <= 0.0) {
      throw std::runtime_error("max_stereo_time_diff_sec must be positive");
    }
    if (sdk_frame_timeout_ms_ <= 0) {
      throw std::runtime_error("sdk_frame_timeout_ms must be positive");
    }
    if (max_image_queue_size_ < 1 || max_imu_queue_size_ < 1 || max_path_size_ < 1) {
      throw std::runtime_error("queue and path size parameters must be positive");
    }
    if (max_imu_window_sec_ <= 0.0) {
      throw std::runtime_error("max_imu_window_sec must be positive");
    }
  }

  void generate_orbslam3_settings()
  {
    if (!calibration_ready_) {
      throw std::runtime_error(
              "ORB-SLAM3 VIO calibration is marked not ready; fill camera_imu_calibration in " +
              config_file_ + " and set orbslam3.calibration_ready: true");
    }

    const std::filesystem::path output_path(generated_settings_file_);
    std::filesystem::create_directories(output_path.parent_path());

    std::ofstream stream(output_path);
    if (!stream.is_open()) {
      throw std::runtime_error("failed to create ORB-SLAM3 settings file: " +
          generated_settings_file_);
    }

    stream << "%YAML:1.0\n\n";

    const std::vector<std::string> preferred_keys = {
      "File.version",
      "Camera.type",
      "Camera1.fx",
      "Camera1.fy",
      "Camera1.cx",
      "Camera1.cy",
      "Camera1.k1",
      "Camera1.k2",
      "Camera1.k3",
      "Camera1.k4",
      "Camera1.overlappingBegin",
      "Camera1.overlappingEnd",
      "Camera2.fx",
      "Camera2.fy",
      "Camera2.cx",
      "Camera2.cy",
      "Camera2.k1",
      "Camera2.k2",
      "Camera2.k3",
      "Camera2.k4",
      "Camera2.overlappingBegin",
      "Camera2.overlappingEnd",
      "Stereo.T_c1_c2",
      "Stereo.ThDepth",
      "Camera.width",
      "Camera.height",
      "Camera.fps",
      "Camera.RGB",
      "Camera.newWidth",
      "Camera.newHeight",
      "IMU.T_b_c1",
      "IMU.NoiseGyro",
      "IMU.NoiseAcc",
      "IMU.GyroWalk",
      "IMU.AccWalk",
      "IMU.Frequency",
      "IMU.InsertKFsWhenLost",
      "ORBextractor.nFeatures",
      "ORBextractor.scaleFactor",
      "ORBextractor.nLevels",
      "ORBextractor.iniThFAST",
      "ORBextractor.minThFAST",
      "Viewer.KeyFrameSize",
      "Viewer.KeyFrameLineWidth",
      "Viewer.GraphLineWidth",
      "Viewer.PointSize",
      "Viewer.CameraSize",
      "Viewer.CameraLineWidth",
      "Viewer.ViewpointX",
      "Viewer.ViewpointY",
      "Viewer.ViewpointZ",
      "Viewer.ViewpointF",
      "Viewer.imageViewScale",
      "System.LoadAtlasFromFile",
      "System.SaveAtlasToFile",
      "System.thFarPoints",
      "loopClosing",
    };

    std::set<std::string> written;
    for (const auto & key : preferred_keys) {
      const YAML::Node value = calibration_section_[key];
      if (!value) {
        continue;
      }
      write_opencv_setting(stream, key, value);
      written.insert(key);
    }

    for (const auto & entry : calibration_section_) {
      const std::string key = entry.first.as<std::string>();
      if (written.find(key) != written.end()) {
        continue;
      }
      write_opencv_setting(stream, key, entry.second);
    }
  }

  void write_opencv_setting(
    std::ostream & stream, const std::string & key,
    const YAML::Node & value)
  {
    if (value.IsMap()) {
      write_opencv_matrix(stream, key, value);
      return;
    }
    if (value.IsScalar()) {
      write_opencv_scalar(stream, key, value);
      return;
    }
    throw std::runtime_error("unsupported OpenCV setting value for key: " + key);
  }

  void initialize_orbslam3()
  {
    if (!std::filesystem::is_directory(library_root_)) {
      throw std::runtime_error("ORB-SLAM3 library_root does not exist: " + library_root_);
    }
    if (!std::filesystem::is_regular_file(vocabulary_file_)) {
      throw std::runtime_error("ORB-SLAM3 vocabulary file does not exist: " + vocabulary_file_);
    }
    if (!std::filesystem::is_regular_file(generated_settings_file_)) {
      throw std::runtime_error("generated ORB-SLAM3 settings file does not exist: " +
              generated_settings_file_);
    }

    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_file_,
      generated_settings_file_,
      sensor_mode_ == SlamSensorMode::kImuStereo ?
      ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO,
      use_viewer_);
    RCLCPP_INFO(get_logger(), "Initialized ORB-SLAM3 sensor mode: %s", sensor_mode_name(sensor_mode_));
  }

  void create_ros_interfaces()
  {
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    if (publish_nav_odom_) {
      nav_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(nav_odom_topic_, 20);
    }
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      path_topic_,
      rclcpp::QoS(1).transient_local().reliable());
    camera_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      camera_pose_topic_,
      20);
    tracked_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      tracked_points_topic_,
      5);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    rclcpp::SubscriptionOptions options;
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = callback_group_;

    if (sensor_mode_ == SlamSensorMode::kImuStereo) {
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr message) {handle_imu(message);},
        options);
    }
    if (image_input_mode_ == ImageInputMode::kSdkDirect) {
      return;
    }
    left_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      left_image_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr message) {handle_left_image(message);},
      options);
    right_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      right_image_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr message) {handle_right_image(message);},
      options);
  }

  std::string left_input_name() const
  {
    return image_input_mode_ == ImageInputMode::kSdkDirect ? sdk_left_device_ : left_image_topic_;
  }

  std::string right_input_name() const
  {
    return image_input_mode_ == ImageInputMode::kSdkDirect ? sdk_right_device_ : right_image_topic_;
  }

  void start_direct_sdk_capture()
  {
    if (sdk_) {
      return;
    }
    sdk_ = std::make_unique<hanmole_sdk::HanmoleSdk>(sdk_config_);
    try {
      sdk_->configure(sdk_left_module_);
      sdk_->configure(sdk_right_module_);
      sdk_->activate(sdk_left_device_);
      sdk_->activate(sdk_right_device_);
    } catch (...) {
      cleanup_direct_sdk();
      throw;
    }

    stop_sdk_capture_.store(false, std::memory_order_release);
    direct_pair_thread_ = std::thread(
      &Orbslam3StereoInertialNode::direct_pair_processing_loop,
      this);
    left_sdk_thread_ = std::thread(
      &Orbslam3StereoInertialNode::direct_sdk_capture_loop,
      this,
      sdk_left_device_,
      true);
    right_sdk_thread_ = std::thread(
      &Orbslam3StereoInertialNode::direct_sdk_capture_loop,
      this,
      sdk_right_device_,
      false);
  }

  void stop_direct_sdk_capture()
  {
    stop_sdk_capture_.store(true, std::memory_order_release);
    direct_pair_condition_.notify_all();
    if (left_sdk_thread_.joinable()) {
      left_sdk_thread_.join();
    }
    if (right_sdk_thread_.joinable()) {
      right_sdk_thread_.join();
    }
    if (direct_pair_thread_.joinable()) {
      direct_pair_thread_.join();
    }
    cleanup_direct_sdk();
  }

  void cleanup_direct_sdk()
  {
    if (!sdk_) {
      return;
    }
    try {
      sdk_->deactivate(sdk_left_device_);
    } catch (...) {
    }
    try {
      sdk_->deactivate(sdk_right_device_);
    } catch (...) {
    }
    try {
      sdk_->cleanup(sdk_left_module_);
    } catch (...) {
    }
    try {
      sdk_->cleanup(sdk_right_module_);
    } catch (...) {
    }
    sdk_.reset();
  }

  void direct_sdk_capture_loop(std::string device_name, bool left)
  {
    const auto timeout = std::chrono::milliseconds(sdk_frame_timeout_ms_);
    while (rclcpp::ok() && !stop_sdk_capture_.load(std::memory_order_acquire)) {
      try {
        DirectImageFrame image_frame;
        const auto sdk_frame = sdk_->camera.get_latest_blocking(device_name, timeout);
        image_frame = make_direct_image_frame(*sdk_frame);
        std::optional<DirectImagePair> pair;
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          auto & queue = left ? direct_left_queue_ : direct_right_queue_;
          queue.push_back(std::move(image_frame));
          if (left) {
            ++direct_left_frames_;
          } else {
            ++direct_right_frames_;
          }
          log_direct_capture_locked(left, queue.back());
          trim_queue(queue, direct_image_queue_limit());
          pair = take_next_direct_pair();
        }
        if (pair.has_value()) {
          enqueue_direct_pair(std::move(*pair));
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "direct SDK camera read failed for %s: %s",
          device_name.c_str(),
          exception.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void enqueue_direct_pair(DirectImagePair pair)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      direct_pair_queue_.push_back(std::move(pair));
      while (static_cast<int>(direct_pair_queue_.size()) > kMaxDirectPairQueueSize) {
        direct_pair_queue_.pop_front();
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "ORB-SLAM3 sdk_direct dropped queued stereo pair because tracking is behind");
      }
    }
    direct_pair_condition_.notify_one();
  }

  void direct_pair_processing_loop()
  {
    while (rclcpp::ok()) {
      DirectImagePair pair;
      std::size_t dropped_pairs = 0U;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        direct_pair_condition_.wait(lock, [this]() {
            return stop_sdk_capture_.load(std::memory_order_acquire) || !direct_pair_queue_.empty();
          });
        if (direct_pair_queue_.empty()) {
          if (stop_sdk_capture_.load(std::memory_order_acquire)) {
            break;
          }
          continue;
        }
        while (direct_pair_queue_.size() > 1U) {
          direct_pair_queue_.pop_front();
          ++dropped_pairs;
        }
        pair = std::move(direct_pair_queue_.back());
        direct_pair_queue_.pop_back();
      }
      if (dropped_pairs > 0U) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "ORB-SLAM3 sdk_direct dropped %zu stale queued stereo pair(s); tracking is behind",
          dropped_pairs);
      }
      process_direct_stereo_pair(pair);
    }
  }

  DirectImageFrame make_direct_image_frame(const hanmole_sdk::ByteFrame & sdk_frame)
  {
    const CameraPayloadHeader payload_header = parse_camera_payload_header(sdk_frame.payload);
    const std::uint64_t capture_timestamp_ns =
      sdk_frame.header.timestamp_ns != 0U ? sdk_frame.header.timestamp_ns : payload_header.timestamp_ns;
    if (capture_timestamp_ns == 0U) {
      throw std::runtime_error("camera SDK frame does not contain capture timestamp");
    }

    DirectImageFrame frame;
    frame.width = payload_header.width;
    frame.height = payload_header.height;
    frame.stride_bytes = payload_header.stride_bytes;
    frame.pixel_format = payload_header.pixel_format;
    frame.stamp = ros_stamp_from_sdk_timestamp(capture_timestamp_ns);
    frame.capture_timestamp_ns = capture_timestamp_ns;
    frame.seq = sdk_frame.header.seq;

    const auto image_size = static_cast<std::size_t>(payload_header.image_size);
    if (image_size == 0U) {
      throw std::runtime_error("camera SDK payload has empty image");
    }
    const auto * image = reinterpret_cast<const std::uint8_t *>(
      sdk_frame.payload.data() + sizeof(CameraPayloadHeader));
    frame.image_payload.assign(image, image + image_size);
    return frame;
  }

  builtin_interfaces::msg::Time ros_stamp_from_sdk_timestamp(std::uint64_t capture_timestamp_ns)
  {
    const auto capture_ns = static_cast<std::int64_t>(capture_timestamp_ns);
    std::int64_t offset_ns = 0;
    {
      std::lock_guard<std::mutex> lock(sdk_time_mutex_);
      if (!sdk_to_ros_time_offset_ns_.has_value()) {
        sdk_to_ros_time_offset_ns_ = now().nanoseconds() - capture_ns;
      }
      offset_ns = *sdk_to_ros_time_offset_ns_;
    }

    std::int64_t ros_ns = capture_ns + offset_ns;
    if (ros_ns < 0) {
      ros_ns = 0;
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ros_ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ros_ns % 1000000000LL);
    return stamp;
  }

  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const auto & acc = message->linear_acceleration;
    const auto & gyro = message->angular_velocity;
    if (!is_finite(acc.x) || !is_finite(acc.y) || !is_finite(acc.z) ||
      !is_finite(gyro.x) || !is_finite(gyro.y) || !is_finite(gyro.z))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "dropping IMU sample with NaN/Inf");
      return;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    imu_queue_.emplace_back(
      static_cast<float>(acc.x),
      static_cast<float>(acc.y),
      static_cast<float>(acc.z),
      static_cast<float>(gyro.x),
      static_cast<float>(gyro.y),
      static_cast<float>(gyro.z),
      stamp_to_seconds(message->header.stamp));
    trim_queue(imu_queue_, max_imu_queue_size_);
  }

  void handle_left_image(const sensor_msgs::msg::Image::SharedPtr message)
  {
    std::optional<ImagePair> pair;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      left_queue_.push_back(message);
      trim_queue(left_queue_, max_image_queue_size_);
      pair = take_next_stereo_pair();
    }
    if (pair.has_value()) {
      process_stereo_pair(*pair);
    }
  }

  void handle_right_image(const sensor_msgs::msg::Image::SharedPtr message)
  {
    std::optional<ImagePair> pair;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      right_queue_.push_back(message);
      trim_queue(right_queue_, max_image_queue_size_);
      pair = take_next_stereo_pair();
    }
    if (pair.has_value()) {
      process_stereo_pair(*pair);
    }
  }

  template<typename T>
  void trim_queue(std::deque<T> & queue, int max_size)
  {
    while (static_cast<int>(queue.size()) > max_size) {
      queue.pop_front();
    }
  }

  int direct_image_queue_limit() const
  {
    return std::min(max_image_queue_size_, kMaxDirectImageQueueSize);
  }

  struct ImagePair
  {
    sensor_msgs::msg::Image::SharedPtr left;
    sensor_msgs::msg::Image::SharedPtr right;
  };

  std::optional<ImagePair> take_next_stereo_pair()
  {
    while (!left_queue_.empty() && !right_queue_.empty()) {
      const double left_time = stamp_to_seconds(left_queue_.front()->header.stamp);
      auto best = right_queue_.begin();
      double best_delta = std::numeric_limits<double>::max();
      for (auto it = right_queue_.begin(); it != right_queue_.end(); ++it) {
        const double delta = std::abs(stamp_to_seconds((*it)->header.stamp) - left_time);
        if (delta < best_delta) {
          best_delta = delta;
          best = it;
        }
      }

      if (best_delta <= max_stereo_time_diff_sec_) {
        ImagePair pair{left_queue_.front(), *best};
        left_queue_.pop_front();
        right_queue_.erase(best);
        return pair;
      }

      const double right_front_time = stamp_to_seconds(right_queue_.front()->header.stamp);
      if (left_time < right_front_time) {
        left_queue_.pop_front();
      } else {
        right_queue_.pop_front();
      }
    }
    return std::nullopt;
  }

  std::optional<DirectImagePair> take_next_direct_pair()
  {
    while (!direct_left_queue_.empty() && !direct_right_queue_.empty()) {
      const double left_time = stamp_to_seconds(direct_left_queue_.front().stamp);
      auto best = direct_right_queue_.begin();
      double best_delta = std::numeric_limits<double>::max();
      for (auto it = direct_right_queue_.begin(); it != direct_right_queue_.end(); ++it) {
        const double delta = std::abs(stamp_to_seconds(it->stamp) - left_time);
        if (delta < best_delta) {
          best_delta = delta;
          best = it;
        }
      }

      if (best_delta <= max_stereo_time_diff_sec_) {
        DirectImagePair pair{std::move(direct_left_queue_.front()), std::move(*best)};
        direct_left_queue_.pop_front();
        direct_right_queue_.erase(best);
        return pair;
      }

      const double right_front_time = stamp_to_seconds(direct_right_queue_.front().stamp);
      if (left_time < right_front_time) {
        record_unmatched_direct_pair_drop(
          "left",
          best_delta,
          direct_left_queue_.front().seq,
          best->seq,
          direct_left_queue_.size(),
          direct_right_queue_.size());
        direct_left_queue_.pop_front();
      } else {
        record_unmatched_direct_pair_drop(
          "right",
          best_delta,
          direct_left_queue_.front().seq,
          direct_right_queue_.front().seq,
          direct_left_queue_.size(),
          direct_right_queue_.size());
        direct_right_queue_.pop_front();
      }
    }
    return std::nullopt;
  }

  void process_stereo_pair(const ImagePair & pair)
  {
    std::lock_guard<std::mutex> tracking_lock(tracking_mutex_);

    const double timestamp = stamp_to_seconds(pair.left->header.stamp);
    const double right_timestamp = stamp_to_seconds(pair.right->header.stamp);
    const double pair_delta = std::abs(timestamp - right_timestamp);
    if (pair_delta > max_stereo_time_diff_sec_) {
      record_rejected_pair("ros_topics", pair_delta, 0U, 0U);
      return;
    }

    try {
      cv::Mat left = image_to_cv_mat(*pair.left);
      cv::Mat right = image_to_cv_mat(*pair.right);

      Sophus::SE3f T_c1_w;
      if (sensor_mode_ == SlamSensorMode::kImuStereo) {
        if (reset_if_imu_window_too_large("ros_topics", timestamp)) {
          return;
        }
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements = take_imu_measurements(timestamp);
        log_imu_window("ros_topics", timestamp, imu_measurements);
        T_c1_w = slam_->TrackStereo(left, right, timestamp, imu_measurements);
      } else {
        T_c1_w = slam_->TrackStereo(left, right, timestamp);
      }
      last_tracking_time_ = timestamp;

      const int tracking_state = slam_->GetTrackingState();
      const bool publishable = tracking_state_is_publishable(tracking_state);
      record_tracking_diagnostics(
        "ros_topics", pair_delta, left, right, tracking_state, publishable, 0U, 0U);
      if (!publishable) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "ORB-SLAM3 tracking state is not publishable: %d",
          tracking_state);
        return;
      }

      const Eigen::Matrix4f T_w_c1 = T_c1_w.inverse().matrix();
      const Eigen::Matrix4f T_w_b = T_w_c1 * T_c1_b_;
      publish_outputs(pair.left->header.stamp, T_w_b, T_w_c1);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "ORB-SLAM3 frame processing failed: %s",
        exception.what());
    }
  }

  void process_direct_stereo_pair(const DirectImagePair & pair)
  {
    std::lock_guard<std::mutex> tracking_lock(tracking_mutex_);

    const double timestamp = stamp_to_seconds(pair.left.stamp);
    const double right_timestamp = stamp_to_seconds(pair.right.stamp);
    const double pair_delta = std::abs(timestamp - right_timestamp);
    if (pair_delta > max_stereo_time_diff_sec_) {
      record_rejected_pair("sdk_direct", pair_delta, pair.left.seq, pair.right.seq);
      return;
    }

    try {
      cv::Mat left_image = decode_direct_image_frame(pair.left);
      cv::Mat right_image = decode_direct_image_frame(pair.right);

      Sophus::SE3f T_c1_w;
      if (sensor_mode_ == SlamSensorMode::kImuStereo) {
        if (reset_if_imu_window_too_large("sdk_direct", timestamp)) {
          return;
        }
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements = take_imu_measurements(timestamp);
        log_imu_window("sdk_direct", timestamp, imu_measurements);
        T_c1_w = slam_->TrackStereo(left_image, right_image, timestamp, imu_measurements);
      } else {
        T_c1_w = slam_->TrackStereo(left_image, right_image, timestamp);
      }
      last_tracking_time_ = timestamp;

      const int tracking_state = slam_->GetTrackingState();
      const bool publishable = tracking_state_is_publishable(tracking_state);
      record_tracking_diagnostics(
        "sdk_direct",
        pair_delta,
        left_image,
        right_image,
        tracking_state,
        publishable,
        pair.left.seq,
        pair.right.seq);
      if (!publishable) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "ORB-SLAM3 tracking state is not publishable: %d",
          tracking_state);
        return;
      }

      const Eigen::Matrix4f T_w_c1 = T_c1_w.inverse().matrix();
      const Eigen::Matrix4f T_w_b = T_w_c1 * T_c1_b_;
      publish_outputs(pair.left.stamp, T_w_b, T_w_c1);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "ORB-SLAM3 direct SDK frame processing failed: %s",
        exception.what());
    }
  }

  bool tracking_state_is_publishable(int tracking_state) const
  {
    return tracking_state == kTrackingOk ||
           tracking_state == kTrackingRecentlyLost ||
           tracking_state == kTrackingOkKlt;
  }

  void log_direct_capture_locked(bool left, const DirectImageFrame & frame)
  {
    const auto left_frames = direct_left_frames_;
    const auto right_frames = direct_right_frames_;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "ORB-SLAM3 direct SDK capture left_frames=%llu right_frames=%llu side=%s seq=%llu "
      "stamp=%.6f image=%dx%d queues=%zu/%zu",
      static_cast<unsigned long long>(left_frames),
      static_cast<unsigned long long>(right_frames),
      left ? "left" : "right",
      static_cast<unsigned long long>(frame.seq),
      stamp_to_seconds(frame.stamp),
      static_cast<int>(frame.width),
      static_cast<int>(frame.height),
      direct_left_queue_.size(),
      direct_right_queue_.size());
  }

  void record_unmatched_direct_pair_drop(
    const char * dropped_side,
    double best_delta,
    std::uint64_t left_seq,
    std::uint64_t right_seq,
    std::size_t left_queue_size,
    std::size_t right_queue_size)
  {
    ++unmatched_direct_pair_drops_;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "ORB-SLAM3 sdk_direct dropped unmatched %s frame: best_dt_ms=%.3f threshold_ms=%.3f "
      "drops=%llu seq=%llu/%llu queues=%zu/%zu",
      dropped_side,
      best_delta * 1000.0,
      max_stereo_time_diff_sec_ * 1000.0,
      static_cast<unsigned long long>(unmatched_direct_pair_drops_),
      static_cast<unsigned long long>(left_seq),
      static_cast<unsigned long long>(right_seq),
      left_queue_size,
      right_queue_size);
  }

  void record_rejected_pair(
    const char * source,
    double pair_delta,
    std::uint64_t left_seq,
    std::uint64_t right_seq)
  {
    ++rejected_stereo_pairs_;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "ORB-SLAM3 %s rejected stereo pair: dt_ms=%.3f threshold_ms=%.3f rejected=%llu "
      "seq=%llu/%llu",
      source,
      pair_delta * 1000.0,
      max_stereo_time_diff_sec_ * 1000.0,
      static_cast<unsigned long long>(rejected_stereo_pairs_),
      static_cast<unsigned long long>(left_seq),
      static_cast<unsigned long long>(right_seq));
  }

  void record_tracking_diagnostics(
    const char * source,
    double pair_delta,
    const cv::Mat & left,
    const cv::Mat & right,
    int tracking_state,
    bool publishable,
    std::uint64_t left_seq,
    std::uint64_t right_seq)
  {
    ++processed_stereo_pairs_;
    if (tracking_state >= 0 &&
      static_cast<std::size_t>(tracking_state) < tracking_state_counts_.size())
    {
      ++tracking_state_counts_[static_cast<std::size_t>(tracking_state)];
    } else {
      ++tracking_state_other_count_;
    }
    if (publishable) {
      ++publishable_tracking_frames_;
    } else {
      ++unpublishable_tracking_frames_;
    }
    max_pair_delta_ms_ = std::max(max_pair_delta_ms_, pair_delta * 1000.0);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "ORB-SLAM3 diag source=%s pairs=%llu rejected=%llu publishable=%llu "
      "unpublishable=%llu state_counts[OK=%llu RECENTLY_LOST=%llu LOST=%llu OK_KLT=%llu "
      "other=%llu] last_state=%d last_dt_ms=%.3f max_dt_ms=%.3f left=%dx%d right=%dx%d "
      "seq=%llu/%llu",
      source,
      static_cast<unsigned long long>(processed_stereo_pairs_),
      static_cast<unsigned long long>(rejected_stereo_pairs_),
      static_cast<unsigned long long>(publishable_tracking_frames_),
      static_cast<unsigned long long>(unpublishable_tracking_frames_),
      static_cast<unsigned long long>(tracking_state_counts_[kTrackingOk]),
      static_cast<unsigned long long>(tracking_state_counts_[kTrackingRecentlyLost]),
      static_cast<unsigned long long>(tracking_state_counts_[4]),
      static_cast<unsigned long long>(tracking_state_counts_[kTrackingOkKlt]),
      static_cast<unsigned long long>(tracking_state_other_count_),
      tracking_state,
      pair_delta * 1000.0,
      max_pair_delta_ms_,
      left.cols,
      left.rows,
      right.cols,
      right.rows,
      static_cast<unsigned long long>(left_seq),
      static_cast<unsigned long long>(right_seq));
  }

  void log_imu_window(
    const char * source,
    double image_timestamp,
    const std::vector<ORB_SLAM3::IMU::Point> & imu_measurements)
  {
    const double last_image_timestamp = last_tracking_time_.value_or(
      std::numeric_limits<double>::quiet_NaN());

    if (imu_measurements.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "ORB-SLAM3 IMU window empty source=%s image_t=%.6f last_image_t=%.6f",
        source,
        image_timestamp,
        last_image_timestamp);
      return;
    }

    const double imu_first = imu_measurements.front().t;
    const double imu_last = imu_measurements.back().t;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "ORB-SLAM3 IMU window source=%s count=%zu image_t=%.6f last_image_t=%.6f "
      "imu_first=%.6f imu_last=%.6f span_ms=%.3f first_after_last_ms=%.3f "
      "image_after_last_imu_ms=%.3f",
      source,
      imu_measurements.size(),
      image_timestamp,
      last_image_timestamp,
      imu_first,
      imu_last,
      (imu_last - imu_first) * 1000.0,
      (imu_first - last_image_timestamp) * 1000.0,
      (image_timestamp - imu_last) * 1000.0);
  }

  std::vector<ORB_SLAM3::IMU::Point> take_imu_measurements(double image_timestamp)
  {
    std::vector<ORB_SLAM3::IMU::Point> result;
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (!last_tracking_time_.has_value()) {
      while (!imu_queue_.empty() && imu_queue_.front().t <= image_timestamp) {
        imu_queue_.pop_front();
      }
      return result;
    }

    while (!imu_queue_.empty() && imu_queue_.front().t <= *last_tracking_time_) {
      imu_queue_.pop_front();
    }
    while (!imu_queue_.empty() && imu_queue_.front().t <= image_timestamp) {
      result.push_back(imu_queue_.front());
      imu_queue_.pop_front();
    }
    return result;
  }

  bool reset_if_imu_window_too_large(const char * source, double image_timestamp)
  {
    if (!last_tracking_time_.has_value()) {
      return false;
    }

    const double last_image_timestamp = *last_tracking_time_;
    const double image_gap_sec = image_timestamp - last_image_timestamp;
    if (image_gap_sec > 0.0 && image_gap_sec <= max_imu_window_sec_) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!imu_queue_.empty() && imu_queue_.front().t <= image_timestamp) {
        imu_queue_.pop_front();
      }
    }

    slam_->ResetActiveMap();
    last_tracking_time_.reset();
    path_.poses.clear();
    ++large_imu_window_resets_;

    RCLCPP_WARN(
      get_logger(),
      "ORB-SLAM3 reset active map and skipped %s stereo pair because IMU/image window is invalid: "
      "image_t=%.6f last_image_t=%.6f gap_ms=%.3f limit_ms=%.3f resets=%llu",
      source,
      image_timestamp,
      last_image_timestamp,
      image_gap_sec * 1000.0,
      max_imu_window_sec_ * 1000.0,
      static_cast<unsigned long long>(large_imu_window_resets_));
    return true;
  }

  void publish_outputs(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Matrix4f & T_w_b,
    const Eigen::Matrix4f & T_w_c1)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose = eigen_pose_to_msg(T_w_b);
    set_default_pose_covariance(odom);
    odom_publisher_->publish(odom);

    if (publish_nav_odom_ && nav_odom_publisher_) {
      const Eigen::Matrix4f T_w_base = T_w_c1 * T_c1_base_;
      const Eigen::Matrix4f T_nav_base = T_nav_vio_ * T_w_base;
      nav_msgs::msg::Odometry nav_odom;
      nav_odom.header.stamp = stamp;
      nav_odom.header.frame_id = nav_frame_id_;
      nav_odom.child_frame_id = nav_child_frame_id_;
      nav_odom.pose.pose = project_nav_odom_2d_ ?
        eigen_pose_to_planar_msg(T_nav_base) : eigen_pose_to_msg(T_nav_base);
      set_default_pose_covariance(nav_odom);
      nav_odom_publisher_->publish(nav_odom);
    }

    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header = odom.header;
    base_pose.pose = odom.pose.pose;
    path_.header = odom.header;
    path_.poses.push_back(base_pose);
    if (static_cast<int>(path_.poses.size()) > max_path_size_) {
      path_.poses.erase(path_.poses.begin());
    }
    path_publisher_->publish(path_);

    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = odom.header;
    camera_pose.pose = eigen_pose_to_msg(T_w_c1);
    camera_pose_publisher_->publish(camera_pose);

    publish_tracked_points(stamp);
    publish_tf_if_enabled(stamp, odom.pose.pose);
  }

  void set_default_pose_covariance(nav_msgs::msg::Odometry & odom) const
  {
    std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[14] = 0.10;
    odom.pose.covariance[21] = 0.02;
    odom.pose.covariance[28] = 0.02;
    odom.pose.covariance[35] = 0.05;
  }

  void publish_tf_if_enabled(
    const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::Pose & pose)
  {
    if (!tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_tracked_points(const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_tracked_points_ || !tracked_points_publisher_) {
      return;
    }
    if (tracked_points_publisher_->get_subscription_count() == 0U) {
      return;
    }

    const auto map_points = slam_->GetTrackedMapPoints();
    std::vector<Eigen::Vector3f> points;
    points.reserve(map_points.size());
    for (auto * map_point : map_points) {
      if (map_point == nullptr) {
        continue;
      }
      points.push_back(map_point->GetWorldPos());
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & point : points) {
      *iter_x = point.x();
      *iter_y = point.y();
      *iter_z = point.z();
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    tracked_points_publisher_->publish(cloud);
  }

  std::string config_file_;
  std::string vocabulary_file_;
  std::string generated_settings_file_;
  std::string library_root_;
  bool calibration_ready_{false};
  SlamSensorMode sensor_mode_{SlamSensorMode::kImuStereo};
  bool use_viewer_{false};
  bool publish_tracked_points_{false};
  YAML::Node calibration_section_;
  Eigen::Matrix4f T_b_c1_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f T_c1_b_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f T_base_c1_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f T_c1_base_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f T_nav_vio_{Eigen::Matrix4f::Identity()};

  ImageInputMode image_input_mode_{ImageInputMode::kSdkDirect};
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string sdk_config_{"v0_1"};
  std::string sdk_left_module_{"mod_camera_head_left"};
  std::string sdk_right_module_{"mod_camera_head_right"};
  std::string sdk_left_device_{"camera_head_left"};
  std::string sdk_right_device_{"camera_head_right"};
  std::string imu_topic_;
  std::string odom_topic_;
  bool publish_nav_odom_{false};
  std::string nav_odom_topic_{"/vio/odom_2d"};
  std::string nav_frame_id_{"vio_odom"};
  std::string nav_child_frame_id_{"base_footprint"};
  bool project_nav_odom_2d_{true};
  std::string path_topic_;
  std::string camera_pose_topic_;
  std::string tracked_points_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  double max_stereo_time_diff_sec_{0.01};
  double max_imu_window_sec_{0.25};
  int max_image_queue_size_{10};
  int max_imu_queue_size_{2000};
  int sdk_frame_timeout_ms_{1000};
  int max_path_size_{2000};
  bool publish_tf_{false};

  std::unique_ptr<hanmole_sdk::HanmoleSdk> sdk_;
  std::thread left_sdk_thread_;
  std::thread right_sdk_thread_;
  std::thread direct_pair_thread_;
  std::atomic<bool> stop_sdk_capture_{false};

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_points_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex queue_mutex_;
  std::mutex tracking_mutex_;
  std::condition_variable direct_pair_condition_;
  std::mutex sdk_time_mutex_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> left_queue_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> right_queue_;
  std::deque<DirectImageFrame> direct_left_queue_;
  std::deque<DirectImageFrame> direct_right_queue_;
  std::deque<DirectImagePair> direct_pair_queue_;
  std::deque<ORB_SLAM3::IMU::Point> imu_queue_;
  std::optional<double> last_tracking_time_;
  std::optional<std::int64_t> sdk_to_ros_time_offset_ns_;
  std::uint64_t processed_stereo_pairs_{0};
  std::uint64_t rejected_stereo_pairs_{0};
  std::uint64_t unmatched_direct_pair_drops_{0};
  std::uint64_t direct_left_frames_{0};
  std::uint64_t direct_right_frames_{0};
  std::uint64_t publishable_tracking_frames_{0};
  std::uint64_t unpublishable_tracking_frames_{0};
  std::uint64_t large_imu_window_resets_{0};
  std::uint64_t tracking_state_other_count_{0};
  double max_pair_delta_ms_{0.0};
  std::array<std::uint64_t, 8> tracking_state_counts_{};
  nav_msgs::msg::Path path_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Orbslam3StereoInertialNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("orbslam3_stereo_inertial_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
