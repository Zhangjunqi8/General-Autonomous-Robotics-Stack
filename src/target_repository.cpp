#include "hanmole_navigation/target_repository.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace hanmole_navigation
{

namespace
{

TargetPose parse_pose(const YAML::Node & node)
{
  if (!node.IsMap()) {
    throw std::runtime_error("target pose must be a map with position/orientation");
  }

  const YAML::Node position = node["position"];
  const YAML::Node orientation = node["orientation"];
  if (!position || !position.IsMap() || !orientation || !orientation.IsMap()) {
    throw std::runtime_error("target pose must contain position and orientation maps");
  }

  return TargetPose{
    position["x"].as<double>(),
    position["y"].as<double>(),
    orientation["x"].as<double>(),
    orientation["y"].as<double>(),
    orientation["z"].as<double>(),
    orientation["w"].as<double>(),
  };
}

double yaw_from_target_pose(const TargetPose & pose)
{
  const double siny_cosp = 2.0 * (pose.qw * pose.qz + pose.qx * pose.qy);
  const double cosy_cosp = 1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string escape_json_string(const std::string & value)
{
  std::ostringstream escaped;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\n':
        escaped << "\\n";
        break;
      default:
        escaped << ch;
        break;
    }
  }
  return escaped.str();
}

}  // namespace

void TargetRepository::load_from_file(const std::string & file_path)
{
  YAML::Node root = YAML::LoadFile(file_path);
  if (!root["frame"] || !root["default_group"] || !root["initial_pose_target"] ||
    !root["target_groups"])
  {
    throw std::runtime_error(
            "target file must contain frame, default_group, initial_pose_target and target_groups");
  }

  frame_ = root["frame"].as<std::string>();
  default_group_ = root["default_group"].as<std::string>();
  initial_pose_target_ = root["initial_pose_target"].as<std::string>();
  if (frame_ != "map") {
    throw std::runtime_error("target repository only supports map frame targets");
  }

  target_groups_.clear();
  for (const auto & group_entry : root["target_groups"]) {
    const std::string group_name = group_entry.first.as<std::string>();
    std::vector<TargetEntry> entries;
    for (const auto & target_entry : group_entry.second) {
      TargetEntry entry;
      entry.name = target_entry.first.as<std::string>();
      const YAML::Node poses_node = target_entry.second["poses"];
      if (!poses_node || !poses_node.IsSequence() || poses_node.size() == 0) {
        throw std::runtime_error("each target must provide a non-empty poses sequence");
      }
      for (const auto & pose_node : poses_node) {
        entry.poses.push_back(parse_pose(pose_node));
      }
      entries.push_back(entry);
    }
    target_groups_[group_name] = entries;
  }

  if (!has_group(default_group_)) {
    throw std::runtime_error("default_group does not exist in target_groups");
  }
  if (!find_target(default_group_, initial_pose_target_).has_value()) {
    throw std::runtime_error("initial_pose_target does not exist in default_group");
  }
}

const std::string & TargetRepository::frame() const
{
  return frame_;
}

const std::string & TargetRepository::default_group() const
{
  return default_group_;
}

const std::string & TargetRepository::initial_pose_target() const
{
  return initial_pose_target_;
}

bool TargetRepository::has_group(const std::string & group) const
{
  return target_groups_.find(group) != target_groups_.end();
}

std::vector<std::string> TargetRepository::list_targets(const std::string & group) const
{
  std::vector<std::string> result;
  const auto it = target_groups_.find(group);
  if (it == target_groups_.end()) {
    return result;
  }

  result.reserve(it->second.size());
  for (const auto & entry : it->second) {
    result.push_back(entry.name);
  }
  return result;
}

std::optional<TargetEntry> TargetRepository::find_target(
  const std::string & group,
  const std::string & target_name) const
{
  const auto it = target_groups_.find(group);
  if (it == target_groups_.end()) {
    return std::nullopt;
  }

  for (const auto & entry : it->second) {
    if (entry.name == target_name) {
      return entry;
    }
  }
  return std::nullopt;
}

std::optional<TargetPose> TargetRepository::resolve_target(
  const std::string & group,
  const std::string & target_name,
  std::optional<std::pair<double, double>> current_pose_xy) const
{
  const auto entry = find_target(group, target_name);
  if (!entry.has_value()) {
    return std::nullopt;
  }

  if (!current_pose_xy.has_value() || entry->poses.size() == 1) {
    return entry->poses.front();
  }

  const auto [current_x, current_y] = *current_pose_xy;
  const TargetPose * best_pose = nullptr;
  double best_distance = std::numeric_limits<double>::max();
  for (const auto & pose : entry->poses) {
    const double distance = std::hypot(pose.x - current_x, pose.y - current_y);
    if (distance < best_distance) {
      best_distance = distance;
      best_pose = &pose;
    }
  }

  if (best_pose == nullptr) {
    return std::nullopt;
  }
  return *best_pose;
}

std::string TargetRepository::catalog_json(const std::string & group) const
{
  std::ostringstream stream;
  stream << "{\"frame\":\"" << escape_json_string(frame_) << "\",";
  stream << "\"group\":\"" << escape_json_string(group) << "\",";
  stream << "\"initial_pose_target\":\"" << escape_json_string(initial_pose_target_) << "\",";
  stream << "\"targets\":[";

  const auto names = list_targets(group);
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << "\"" << escape_json_string(names[index]) << "\"";
  }
  stream << "]}";
  return stream.str();
}

std::string TargetRepository::pose_list_json(const std::string & group) const
{
  std::ostringstream stream;
  stream << std::setprecision(17);
  stream << "{";

  const auto group_it = target_groups_.find(group);
  if (group_it != target_groups_.end()) {
    bool first = true;
    for (const auto & entry : group_it->second) {
      if (entry.poses.empty()) {
        continue;
      }

      const auto & pose = entry.poses.front();
      if (!first) {
        stream << ",";
      }
      first = false;

      stream << "\"" << escape_json_string(entry.name) << "\":";
      stream << "[";
      stream << pose.x << "," << pose.y << "," << yaw_from_target_pose(pose);
      stream << "]";
    }
  }

  stream << "}";
  return stream.str();
}

}  // namespace hanmole_navigation
