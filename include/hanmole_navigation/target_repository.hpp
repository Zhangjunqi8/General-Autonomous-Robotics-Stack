#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace hanmole_navigation
{

struct TargetPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TargetEntry
{
  std::string name;
  std::vector<TargetPose> poses;
};

class TargetRepository
{
public:
  void load_from_file(const std::string & file_path);

  const std::string & frame() const;
  const std::string & default_group() const;
  const std::string & initial_pose_target() const;

  bool has_group(const std::string & group) const;
  std::vector<std::string> list_targets(const std::string & group) const;
  std::optional<TargetEntry> find_target(
    const std::string & group,
    const std::string & target_name) const;
  std::optional<TargetPose> resolve_target(
    const std::string & group,
    const std::string & target_name,
    std::optional<std::pair<double, double>> current_pose_xy) const;
  std::string catalog_json(const std::string & group) const;

private:
  std::string frame_{"map"};
  std::string default_group_{"default"};
  std::string initial_pose_target_{"HOME"};
  std::unordered_map<std::string, std::vector<TargetEntry>> target_groups_;
};

}  // namespace hanmole_navigation
