#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <csignal>

#include "hanmole_sdk/api/sdk.hpp"
#include "hanmole_sdk/data/frame.hpp"

namespace
{

constexpr const char * kSdkConfig = "v0_1";
constexpr const char * kLeftModule = "mod_camera_head_left";
constexpr const char * kRightModule = "mod_camera_head_right";
constexpr const char * kLeftDevice = "camera_head_left";
constexpr const char * kRightDevice = "camera_head_right";
constexpr const char * kOutputRoot = "/home/hanmole/calib/v0_1/bags";
constexpr std::chrono::milliseconds kFrameTimeout{1000};
constexpr std::size_t kMaxPendingPerSide = 240U;
constexpr std::uint32_t kCameraPayloadMagic = 0x314D4348U;

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

struct CapturedFrame
{
  std::uint64_t stamp_ns{0};
  std::uint64_t seq{0};
  CameraPayloadHeader payload_header{};
  std::vector<std::byte> payload;
};

struct StereoPair
{
  CapturedFrame left;
  CapturedFrame right;
};

struct RecorderStats
{
  std::uint64_t left_read{0};
  std::uint64_t right_read{0};
  std::uint64_t pairs_written{0};
  std::uint64_t left_unmatched{0};
  std::uint64_t right_unmatched{0};
  std::uint64_t duplicate_left{0};
  std::uint64_t duplicate_right{0};
};

std::atomic_bool g_stop{false};

void handle_signal(int)
{
  g_stop.store(true, std::memory_order_release);
}

std::string now_string()
{
  const std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
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

CapturedFrame make_captured_frame(const hanmole_sdk::ByteFrame & sdk_frame)
{
  CapturedFrame frame;
  frame.payload_header = parse_camera_payload_header(sdk_frame.payload);
  frame.stamp_ns = sdk_frame.header.timestamp_ns != 0U ?
    sdk_frame.header.timestamp_ns : frame.payload_header.timestamp_ns;
  if (frame.stamp_ns == 0U) {
    throw std::runtime_error("camera SDK frame does not contain capture timestamp");
  }
  frame.seq = sdk_frame.header.seq;
  frame.payload = sdk_frame.payload;
  return frame;
}

void write_binary_file(const std::filesystem::path & path, const std::vector<std::byte> & payload)
{
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    throw std::runtime_error("failed to open output file: " + path.string());
  }
  file.write(reinterpret_cast<const char *>(payload.data()), static_cast<std::streamsize>(payload.size()));
  if (!file) {
    throw std::runtime_error("failed to write output file: " + path.string());
  }
}

std::string payload_name(std::uint64_t index)
{
  std::ostringstream stream;
  stream << std::setw(12) << std::setfill('0') << index << ".bin";
  return stream.str();
}

class StrictStereoRecorder
{
public:
  StrictStereoRecorder()
  : output_dir_(std::filesystem::path(kOutputRoot) / ("sdk_stereo_calib_" + now_string())),
    left_dir_(output_dir_ / "left_payload"),
    right_dir_(output_dir_ / "right_payload")
  {
    std::filesystem::create_directories(left_dir_);
    std::filesystem::create_directories(right_dir_);

    pairs_csv_.open(output_dir_ / "pairs.csv", std::ios::trunc);
    unmatched_left_csv_.open(output_dir_ / "unmatched_left.csv", std::ios::trunc);
    unmatched_right_csv_.open(output_dir_ / "unmatched_right.csv", std::ios::trunc);
    if (!pairs_csv_ || !unmatched_left_csv_ || !unmatched_right_csv_) {
      throw std::runtime_error("failed to open recorder csv files under " + output_dir_.string());
    }

    pairs_csv_
      << "index,stamp_ns,left_seq,right_seq,left_payload,right_payload,width,height,"
      << "stride_bytes,pixel_format,fps,left_image_size,right_image_size,"
      << "left_payload_size,right_payload_size\n";
    unmatched_left_csv_ << "stamp_ns,seq,width,height,pixel_format,payload_size,reason\n";
    unmatched_right_csv_ << "stamp_ns,seq,width,height,pixel_format,payload_size,reason\n";
  }

  void run()
  {
    std::cout << "Output: " << output_dir_ << "\n";
    std::cout << "Pairing rule: exact SDK capture timestamp only\n";
    std::cout << "Press Ctrl+C to stop\n";

    sdk_ = std::make_unique<hanmole_sdk::HanmoleSdk>(kSdkConfig);
    try {
      sdk_->configure(kLeftModule);
      sdk_->configure(kRightModule);
      sdk_->activate(kLeftDevice);
      sdk_->activate(kRightDevice);
    } catch (...) {
      cleanup_sdk();
      throw;
    }

    writer_thread_ = std::thread(&StrictStereoRecorder::writer_loop, this);
    left_thread_ = std::thread(&StrictStereoRecorder::capture_loop, this, std::string(kLeftDevice), true);
    right_thread_ = std::thread(&StrictStereoRecorder::capture_loop, this, std::string(kRightDevice), false);

    while (!g_stop.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      report();
    }

    stop();
  }

private:
  void stop()
  {
    g_stop.store(true, std::memory_order_release);
    if (left_thread_.joinable()) {
      left_thread_.join();
    }
    if (right_thread_.joinable()) {
      right_thread_.join();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      flush_pending_unmatched_locked("shutdown");
      writer_stop_ = true;
    }
    condition_.notify_all();
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }

    cleanup_sdk();
    write_session_json();
    report();
    std::cout << "Stopped. Dataset: " << output_dir_ << "\n";
  }

  void cleanup_sdk()
  {
    if (!sdk_) {
      return;
    }
    try {
      sdk_->deactivate(kLeftDevice);
    } catch (...) {
    }
    try {
      sdk_->deactivate(kRightDevice);
    } catch (...) {
    }
    try {
      sdk_->cleanup(kLeftModule);
    } catch (...) {
    }
    try {
      sdk_->cleanup(kRightModule);
    } catch (...) {
    }
    sdk_.reset();
  }

  void capture_loop(const std::string device_name, const bool is_left)
  {
    std::uint64_t last_seq = 0U;
    while (!g_stop.load(std::memory_order_acquire)) {
      try {
        const auto sdk_frame = sdk_->camera.get_latest_blocking(device_name, kFrameTimeout);
        CapturedFrame frame = make_captured_frame(*sdk_frame);
        if (frame.seq != 0U && frame.seq <= last_seq) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (is_left) {
            ++stats_.duplicate_left;
          } else {
            ++stats_.duplicate_right;
          }
          continue;
        }
        last_seq = frame.seq;
        accept_frame(std::move(frame), is_left);
      } catch (const std::exception & exception) {
        if (!g_stop.load(std::memory_order_acquire)) {
          std::cerr << "SDK read failed for " << device_name << ": " << exception.what() << "\n";
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    }
  }

  void accept_frame(CapturedFrame frame, const bool is_left)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_left) {
      ++stats_.left_read;
      auto match = right_pending_.find(frame.stamp_ns);
      if (match != right_pending_.end()) {
        pair_queue_.push_back(StereoPair{std::move(frame), std::move(match->second)});
        right_pending_.erase(match);
        condition_.notify_one();
      } else {
        left_pending_[frame.stamp_ns] = std::move(frame);
        prune_pending_locked(left_pending_, unmatched_left_csv_, stats_.left_unmatched, "no_exact_right_match");
      }
    } else {
      ++stats_.right_read;
      auto match = left_pending_.find(frame.stamp_ns);
      if (match != left_pending_.end()) {
        pair_queue_.push_back(StereoPair{std::move(match->second), std::move(frame)});
        left_pending_.erase(match);
        condition_.notify_one();
      } else {
        right_pending_[frame.stamp_ns] = std::move(frame);
        prune_pending_locked(right_pending_, unmatched_right_csv_, stats_.right_unmatched, "no_exact_left_match");
      }
    }
  }

  void prune_pending_locked(
    std::map<std::uint64_t, CapturedFrame> & pending,
    std::ofstream & unmatched_csv,
    std::uint64_t & unmatched_count,
    const char * reason)
  {
    while (pending.size() > kMaxPendingPerSide) {
      auto oldest = pending.begin();
      write_unmatched(unmatched_csv, oldest->second, reason);
      ++unmatched_count;
      pending.erase(oldest);
    }
  }

  void flush_pending_unmatched_locked(const char * reason)
  {
    for (const auto & entry : left_pending_) {
      write_unmatched(unmatched_left_csv_, entry.second, reason);
      ++stats_.left_unmatched;
    }
    for (const auto & entry : right_pending_) {
      write_unmatched(unmatched_right_csv_, entry.second, reason);
      ++stats_.right_unmatched;
    }
    left_pending_.clear();
    right_pending_.clear();
  }

  void write_unmatched(std::ofstream & stream, const CapturedFrame & frame, const char * reason)
  {
    stream << frame.stamp_ns << ','
           << frame.seq << ','
           << frame.payload_header.width << ','
           << frame.payload_header.height << ','
           << frame.payload_header.pixel_format << ','
           << frame.payload.size() << ','
           << reason << '\n';
  }

  void writer_loop()
  {
    while (true) {
      StereoPair pair;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {return writer_stop_ || !pair_queue_.empty();});
        if (pair_queue_.empty()) {
          if (writer_stop_) {
            break;
          }
          continue;
        }
        pair = std::move(pair_queue_.front());
        pair_queue_.pop_front();
      }
      write_pair(std::move(pair));
    }
  }

  void write_pair(StereoPair pair)
  {
    const std::uint64_t index = next_pair_index_++;
    const std::string left_name = payload_name(index);
    const std::string right_name = payload_name(index);
    const auto left_path = left_dir_ / left_name;
    const auto right_path = right_dir_ / right_name;

    write_binary_file(left_path, pair.left.payload);
    write_binary_file(right_path, pair.right.payload);

    std::lock_guard<std::mutex> lock(mutex_);
    pairs_csv_ << index << ','
               << pair.left.stamp_ns << ','
               << pair.left.seq << ','
               << pair.right.seq << ','
               << "left_payload/" << left_name << ','
               << "right_payload/" << right_name << ','
               << pair.left.payload_header.width << ','
               << pair.left.payload_header.height << ','
               << pair.left.payload_header.stride_bytes << ','
               << pair.left.payload_header.pixel_format << ','
               << pair.left.payload_header.fps << ','
               << pair.left.payload_header.image_size << ','
               << pair.right.payload_header.image_size << ','
               << pair.left.payload.size() << ','
               << pair.right.payload.size() << '\n';
    ++stats_.pairs_written;
  }

  void report()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "left=" << stats_.left_read
              << " right=" << stats_.right_read
              << " pairs=" << stats_.pairs_written
              << " pending_left=" << left_pending_.size()
              << " pending_right=" << right_pending_.size()
              << " queue=" << pair_queue_.size()
              << " unmatched_left=" << stats_.left_unmatched
              << " unmatched_right=" << stats_.right_unmatched
              << "\n";
  }

  void write_session_json()
  {
    std::ofstream session(output_dir_ / "session.json", std::ios::trunc);
    session << "{\n"
            << "  \"format\": \"hanmole_sdk_stereo_calib_v1\",\n"
            << "  \"pairing\": \"exact_sdk_capture_timestamp\",\n"
            << "  \"sdk_config\": \"" << kSdkConfig << "\",\n"
            << "  \"left_module\": \"" << kLeftModule << "\",\n"
            << "  \"right_module\": \"" << kRightModule << "\",\n"
            << "  \"left_device\": \"" << kLeftDevice << "\",\n"
            << "  \"right_device\": \"" << kRightDevice << "\",\n"
            << "  \"left_read\": " << stats_.left_read << ",\n"
            << "  \"right_read\": " << stats_.right_read << ",\n"
            << "  \"pairs_written\": " << stats_.pairs_written << ",\n"
            << "  \"left_unmatched\": " << stats_.left_unmatched << ",\n"
            << "  \"right_unmatched\": " << stats_.right_unmatched << "\n"
            << "}\n";
  }

  std::filesystem::path output_dir_;
  std::filesystem::path left_dir_;
  std::filesystem::path right_dir_;
  std::ofstream pairs_csv_;
  std::ofstream unmatched_left_csv_;
  std::ofstream unmatched_right_csv_;

  std::unique_ptr<hanmole_sdk::HanmoleSdk> sdk_;
  std::thread left_thread_;
  std::thread right_thread_;
  std::thread writer_thread_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool writer_stop_{false};
  std::map<std::uint64_t, CapturedFrame> left_pending_;
  std::map<std::uint64_t, CapturedFrame> right_pending_;
  std::deque<StereoPair> pair_queue_;
  RecorderStats stats_;
  std::uint64_t next_pair_index_{0};
};

}  // namespace

int main()
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  try {
    StrictStereoRecorder recorder;
    recorder.run();
  } catch (const std::exception & exception) {
    std::cerr << "sdk_stereo_calib_recorder failed: " << exception.what() << "\n";
    return 1;
  }
  return 0;
}
