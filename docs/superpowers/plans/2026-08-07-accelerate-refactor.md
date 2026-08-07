# Accelerate Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Consolidate the velocity acceleration-chain headers and sources into `accelerate.hpp` and `accelerate.cpp` without changing existing `cmd_vel_nav` acceleration behavior.

**Architecture:** Keep the two existing ROS nodes and public class names, but move their declarations into one header and their implementations into one source file. Build one component library from `accelerate.cpp`, and build the two existing standalone executables from the same source with compile definitions selecting the correct `main()`.

**Tech Stack:** ROS 2 Jazzy, C++17, ament_cmake, rclcpp, rclcpp_components, geometry_msgs, sensor_msgs, std_msgs, gtest.

---

## File Structure

- Create: `include/hanmole_navigation/accelerate.hpp`
  - Consolidated public declarations for boost-source helpers, mux helpers, `CmdVelBoostSourceNode`, and `CmdVelMuxStampedNode`.
- Create: `src/accelerate.cpp`
  - Consolidated implementation for both nodes.
  - Conditional standalone `main()` entry points.
  - Conditional component registrations.
- Modify: `CMakeLists.txt`
  - Replace the two old component targets with one acceleration component target.
  - Keep standalone executable names unchanged.
  - Link tests against the consolidated component target.
- Modify: `test/test_cmd_vel_boost_chain.cpp`
  - Include `hanmole_navigation/accelerate.hpp`.
- Delete after migration:
  - `include/hanmole_navigation/cmd_vel_boost_source.hpp`
  - `include/hanmole_navigation/cmd_vel_mux_stamped.hpp`
  - `src/cmd_vel_boost_source_node.cpp`
  - `src/cmd_vel_boost_source_main.cpp`
  - `src/cmd_vel_mux_stamped_node.cpp`
  - `src/cmd_vel_mux_stamped_main.cpp`
- No changes:
  - `launch/navigation_launch.py`
  - `config/**/*.yaml`

## Task 1: RED - Move Boost-Chain Test To New Public Header

**Files:**
- Modify: `test/test_cmd_vel_boost_chain.cpp`

- [ ] **Step 1: Update the test include**

Replace:

```cpp
#include "hanmole_navigation/cmd_vel_boost_source.hpp"
#include "hanmole_navigation/cmd_vel_mux_stamped.hpp"
```

with:

```cpp
#include "hanmole_navigation/accelerate.hpp"
```

- [ ] **Step 2: Run the targeted test and verify the expected failure**

Run from `/home/hanmole/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select hanmole_navigation --event-handlers console_direct+ --ctest-args -R test_cmd_vel_boost_chain
```

Expected: FAIL during compilation with a missing header error for `hanmole_navigation/accelerate.hpp`.

## Task 2: GREEN - Add A Temporary Aggregating Header

**Files:**
- Create: `include/hanmole_navigation/accelerate.hpp`
- Test: `test/test_cmd_vel_boost_chain.cpp`

- [ ] **Step 1: Add the minimal header that satisfies the new include**

Create `include/hanmole_navigation/accelerate.hpp`:

```cpp
#ifndef HANMOLE_NAVIGATION__ACCELERATE_HPP_
#define HANMOLE_NAVIGATION__ACCELERATE_HPP_

#include "hanmole_navigation/cmd_vel_boost_source.hpp"
#include "hanmole_navigation/cmd_vel_mux_stamped.hpp"

#endif  // HANMOLE_NAVIGATION__ACCELERATE_HPP_
```

- [ ] **Step 2: Run the targeted test and verify it passes**

Run from `/home/hanmole/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select hanmole_navigation --event-handlers console_direct+ --ctest-args -R test_cmd_vel_boost_chain
```

Expected: PASS for `test_cmd_vel_boost_chain`.

- [ ] **Step 3: Commit the green public-header step**

```bash
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add include/hanmole_navigation/accelerate.hpp test/test_cmd_vel_boost_chain.cpp
git -C /home/hanmole/ros2_ws/src/hanmole_navigation commit -m "test: use accelerate header for boost chain"
```

## Task 3: REFACTOR - Consolidate Header Declarations

**Files:**
- Modify: `include/hanmole_navigation/accelerate.hpp`
- Delete: `include/hanmole_navigation/cmd_vel_boost_source.hpp`
- Delete: `include/hanmole_navigation/cmd_vel_mux_stamped.hpp`
- Test: `test/test_cmd_vel_boost_chain.cpp`

- [ ] **Step 1: Replace the temporary header with the full consolidated declarations**

Use this full file for `include/hanmole_navigation/accelerate.hpp`:

```cpp
#ifndef HANMOLE_NAVIGATION__ACCELERATE_HPP_
#define HANMOLE_NAVIGATION__ACCELERATE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace hanmole_navigation
{

struct BoostSourceConfig
{
  double boost_start_linear_x{0.60};
  double boost_gain{4.00};
  double max_linear_x{1.20};
  double max_abs_linear_y{0.10};
  double max_abs_angular_z{0.15};
  double corridor_lookahead_m{3.0};
  double corridor_half_width{0.40};
};

double boostLinearX(
  double linear_x,
  double boost_start_linear_x,
  double boost_gain,
  double max_linear_x);

bool forwardCorridorIsClear(
  const sensor_msgs::msg::LaserScan & scan,
  double corridor_lookahead_m,
  double corridor_half_width);

bool twistQualifiesForBoost(
  const geometry_msgs::msg::TwistStamped & command,
  const BoostSourceConfig & config);

std::optional<geometry_msgs::msg::TwistStamped> makeBoostedCommandIfAllowed(
  const geometry_msgs::msg::TwistStamped & command,
  const sensor_msgs::msg::LaserScan & scan,
  const BoostSourceConfig & config);

class CmdVelBoostSourceNode final : public rclcpp::Node
{
public:
  explicit CmdVelBoostSourceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool latestScanIsFresh(const rclcpp::Time & now) const;
  void validateConfig() const;

  BoostSourceConfig config_;
  double scan_timeout_{0.30};
  std::string input_topic_;
  std::string output_topic_;
  std::string scan_topic_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::optional<sensor_msgs::msg::LaserScan> latest_scan_;
  rclcpp::Time latest_scan_time_{0, 0, RCL_ROS_TIME};
};

struct StampedMuxInputState
{
  std::string name;
  std::string topic;
  double timeout{0.20};
  int priority{0};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;
  std::optional<geometry_msgs::msg::TwistStamped> last_msg;
  rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
};

bool stampedMuxInputIsFresh(
  const StampedMuxInputState & input,
  const rclcpp::Time & now);

const StampedMuxInputState * selectActiveStampedInput(
  const std::vector<StampedMuxInputState> & inputs,
  const rclcpp::Time & now);

geometry_msgs::msg::TwistStamped makeMuxOutput(const StampedMuxInputState & input);

class CmdVelMuxStampedNode final : public rclcpp::Node
{
public:
  explicit CmdVelMuxStampedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void configureInputs(const std::vector<std::string> & input_names);
  void inputCallback(
    std::size_t input_index,
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publishTimerCallback();
  void publishActiveName(const std::string & active_name);

  std::vector<StampedMuxInputState> inputs_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string output_topic_;
  std::string active_topic_;
  std::string active_name_;
  double publish_rate_{50.0};
};

}  // namespace hanmole_navigation

#endif  // HANMOLE_NAVIGATION__ACCELERATE_HPP_
```

- [ ] **Step 2: Delete the old public headers**

```bash
rm /home/hanmole/ros2_ws/src/hanmole_navigation/include/hanmole_navigation/cmd_vel_boost_source.hpp
rm /home/hanmole/ros2_ws/src/hanmole_navigation/include/hanmole_navigation/cmd_vel_mux_stamped.hpp
```

- [ ] **Step 3: Update source includes temporarily**

Before source consolidation, change these includes so the existing source files still compile:

```cpp
#include "hanmole_navigation/accelerate.hpp"
```

Affected files:

- `src/cmd_vel_boost_source_node.cpp`
- `src/cmd_vel_boost_source_main.cpp`
- `src/cmd_vel_mux_stamped_node.cpp`
- `src/cmd_vel_mux_stamped_main.cpp`

- [ ] **Step 4: Run the targeted test and verify it still passes**

Run from `/home/hanmole/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select hanmole_navigation --event-handlers console_direct+ --ctest-args -R test_cmd_vel_boost_chain
```

Expected: PASS for `test_cmd_vel_boost_chain`.

- [ ] **Step 5: Commit the consolidated header**

```bash
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add include/hanmole_navigation/accelerate.hpp test/test_cmd_vel_boost_chain.cpp src/cmd_vel_boost_source_node.cpp src/cmd_vel_boost_source_main.cpp src/cmd_vel_mux_stamped_node.cpp src/cmd_vel_mux_stamped_main.cpp
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add -u include/hanmole_navigation
git -C /home/hanmole/ros2_ws/src/hanmole_navigation commit -m "refactor: consolidate accelerate header"
```

## Task 4: REFACTOR - Consolidate Implementations Into `accelerate.cpp`

**Files:**
- Create: `src/accelerate.cpp`
- Modify: `CMakeLists.txt`
- Delete:
  - `src/cmd_vel_boost_source_node.cpp`
  - `src/cmd_vel_boost_source_main.cpp`
  - `src/cmd_vel_mux_stamped_node.cpp`
  - `src/cmd_vel_mux_stamped_main.cpp`

- [ ] **Step 1: Create `src/accelerate.cpp`**

The file must include:

```cpp
#include "hanmole_navigation/accelerate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#if defined(HANMOLE_NAVIGATION_ACCELERATE_COMPONENTS)
#include "rclcpp_components/register_node_macro.hpp"
#endif
```

Then move the existing implementation bodies into this order without changing logic:

```cpp
namespace hanmole_navigation
{

// Exact bodies from cmd_vel_boost_source_node.cpp:
// boostLinearX
// forwardCorridorIsClear
// twistQualifiesForBoost
// makeBoostedCommandIfAllowed
// CmdVelBoostSourceNode constructor
// CmdVelBoostSourceNode::validateConfig
// CmdVelBoostSourceNode::cmdVelCallback
// CmdVelBoostSourceNode::scanCallback
// CmdVelBoostSourceNode::latestScanIsFresh

namespace
{

// Exact bodies from cmd_vel_mux_stamped_node.cpp:
// defaultTopicForInput
// defaultTimeoutForInput
// defaultPriorityForInput

}  // namespace

// Exact bodies from cmd_vel_mux_stamped_node.cpp:
// stampedMuxInputIsFresh
// selectActiveStampedInput
// makeMuxOutput
// CmdVelMuxStampedNode constructor
// CmdVelMuxStampedNode::configureInputs
// CmdVelMuxStampedNode::inputCallback
// CmdVelMuxStampedNode::publishTimerCallback
// CmdVelMuxStampedNode::publishActiveName

}  // namespace hanmole_navigation
```

Add the component registrations under the component compile definition:

```cpp
#if defined(HANMOLE_NAVIGATION_ACCELERATE_COMPONENTS)
RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelBoostSourceNode)
RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelMuxStampedNode)
#endif
```

Add the standalone entry points under mutually exclusive compile definitions:

```cpp
#if defined(HANMOLE_NAVIGATION_ACCELERATE_BOOST_MAIN)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelBoostSourceNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

#if defined(HANMOLE_NAVIGATION_ACCELERATE_MUX_MAIN)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelMuxStampedNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
```

- [ ] **Step 2: Update `CMakeLists.txt` acceleration targets**

Replace the old `cmd_vel_boost_source_component`, `cmd_vel_boost_source_node`, `cmd_vel_mux_stamped_component`, and `cmd_vel_mux_stamped_node` target blocks with:

```cmake
add_library(accelerate_component SHARED
  src/accelerate.cpp
)
target_include_directories(accelerate_component PUBLIC include)
target_compile_definitions(
  accelerate_component
  PRIVATE HANMOLE_NAVIGATION_ACCELERATE_COMPONENTS
)
ament_target_dependencies(
  accelerate_component
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)
rclcpp_components_register_nodes(
  accelerate_component
  "hanmole_navigation::CmdVelBoostSourceNode"
  "hanmole_navigation::CmdVelMuxStampedNode"
)

add_executable(cmd_vel_boost_source_node src/accelerate.cpp)
target_include_directories(cmd_vel_boost_source_node PRIVATE include)
target_compile_definitions(
  cmd_vel_boost_source_node
  PRIVATE HANMOLE_NAVIGATION_ACCELERATE_BOOST_MAIN
)
ament_target_dependencies(
  cmd_vel_boost_source_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(cmd_vel_mux_stamped_node src/accelerate.cpp)
target_include_directories(cmd_vel_mux_stamped_node PRIVATE include)
target_compile_definitions(
  cmd_vel_mux_stamped_node
  PRIVATE HANMOLE_NAVIGATION_ACCELERATE_MUX_MAIN
)
ament_target_dependencies(
  cmd_vel_mux_stamped_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
```

In the install targets list, replace:

```cmake
cmd_vel_boost_source_component
cmd_vel_mux_stamped_component
```

with:

```cmake
accelerate_component
```

In the `test_cmd_vel_boost_chain` link list, replace:

```cmake
cmd_vel_boost_source_component
cmd_vel_mux_stamped_component
```

with:

```cmake
accelerate_component
```

- [ ] **Step 3: Delete old source files**

```bash
rm /home/hanmole/ros2_ws/src/hanmole_navigation/src/cmd_vel_boost_source_node.cpp
rm /home/hanmole/ros2_ws/src/hanmole_navigation/src/cmd_vel_boost_source_main.cpp
rm /home/hanmole/ros2_ws/src/hanmole_navigation/src/cmd_vel_mux_stamped_node.cpp
rm /home/hanmole/ros2_ws/src/hanmole_navigation/src/cmd_vel_mux_stamped_main.cpp
```

- [ ] **Step 4: Verify old header and source paths are gone from package references**

Run from `/home/hanmole/ros2_ws/src/hanmole_navigation`:

```bash
rg -n "cmd_vel_boost_source.hpp|cmd_vel_mux_stamped.hpp|cmd_vel_boost_source_node.cpp|cmd_vel_mux_stamped_node.cpp|cmd_vel_boost_source_main.cpp|cmd_vel_mux_stamped_main.cpp" .
```

Expected: no matches.

- [ ] **Step 5: Run targeted boost-chain test**

Run from `/home/hanmole/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select hanmole_navigation --event-handlers console_direct+ --ctest-args -R test_cmd_vel_boost_chain
```

Expected: PASS for `test_cmd_vel_boost_chain`.

- [ ] **Step 6: Commit the source consolidation**

```bash
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add CMakeLists.txt include/hanmole_navigation/accelerate.hpp src/accelerate.cpp test/test_cmd_vel_boost_chain.cpp
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add -u include/hanmole_navigation src
git -C /home/hanmole/ros2_ws/src/hanmole_navigation commit -m "refactor: consolidate accelerate implementation"
```

## Task 5: Full Verification

**Files:**
- Inspect: `launch/navigation_launch.py`
- Inspect: `config/v0_1/nav2_params.yaml`
- Inspect: `CMakeLists.txt`

- [ ] **Step 1: Confirm launch names remain unchanged**

Run from `/home/hanmole/ros2_ws/src/hanmole_navigation`:

```bash
rg -n "cmd_vel_boost_source_node|cmd_vel_mux_stamped_node|CmdVelBoostSourceNode|CmdVelMuxStampedNode|cmd_vel_boost_source|cmd_vel_mux_stamped" launch test config CMakeLists.txt
```

Expected: launch still references `cmd_vel_boost_source_node`, `cmd_vel_mux_stamped_node`, `hanmole_navigation::CmdVelBoostSourceNode`, and `hanmole_navigation::CmdVelMuxStampedNode`.

- [ ] **Step 2: Run the package tests**

Run from `/home/hanmole/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select hanmole_navigation --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all `hanmole_navigation` tests pass.

- [ ] **Step 3: Commit any final test or CMake corrections**

Only run this if Step 2 required corrections:

```bash
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add CMakeLists.txt include/hanmole_navigation/accelerate.hpp src/accelerate.cpp test/test_cmd_vel_boost_chain.cpp
git -C /home/hanmole/ros2_ws/src/hanmole_navigation add -u include/hanmole_navigation src test
git -C /home/hanmole/ros2_ws/src/hanmole_navigation commit -m "fix: preserve accelerate refactor behavior"
```
