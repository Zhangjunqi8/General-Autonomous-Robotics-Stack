#include "hanmole_navigation/accelerate.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace hanmole_navigation
{
namespace
{

std::string defaultTopicForInput(const std::string & name)
{
  return name == "nav_fast" ? "cmd_vel_nav_fast" : "cmd_vel_nav";
}

double defaultTimeoutForInput(const std::string & name)
{
  return name == "nav_fast" ? 0.08 : 0.20;
}

int defaultPriorityForInput(const std::string & name)
{
  return name == "nav_fast" ? 100 : 10;
}

}  // namespace

bool stampedMuxInputIsFresh(
  const StampedMuxInputState & input,
  const rclcpp::Time & now)
{
  if (!input.last_msg.has_value()) {
    return false;
  }
  const double age = (now - input.last_time).seconds();
  return age >= 0.0 && age <= input.timeout;
}

const StampedMuxInputState * selectActiveStampedInput(
  const std::vector<StampedMuxInputState> & inputs,
  const rclcpp::Time & now)
{
  const StampedMuxInputState * selected = nullptr;
  for (const auto & input : inputs) {
    if (!stampedMuxInputIsFresh(input, now)) {
      continue;
    }
    if (selected == nullptr || input.priority > selected->priority) {
      selected = &input;
    }
  }
  return selected;
}

geometry_msgs::msg::TwistStamped makeMuxOutput(const StampedMuxInputState & input)
{
  if (!input.last_msg.has_value()) {
    throw std::runtime_error("cannot make mux output without a message");
  }
  return *input.last_msg;
}

CmdVelMuxStampedNode::CmdVelMuxStampedNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_mux_stamped", options)
{
  output_topic_ = declare_parameter<std::string>("output_topic", "cmd_vel_smoothed");
  active_topic_ = declare_parameter<std::string>("active_topic", "cmd_vel_mux_stamped/active");
  publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
  if (publish_rate_ <= 0.0) {
    throw std::runtime_error("publish_rate must be positive");
  }

  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  active_pub_ = create_publisher<std_msgs::msg::String>(
    active_topic_,
    rclcpp::QoS(1).transient_local());

  const auto input_names = declare_parameter<std::vector<std::string>>(
    "input_names",
    {"nav_fast", "nav"});
  configureInputs(input_names);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  publish_timer_ = create_wall_timer(
    period,
    std::bind(&CmdVelMuxStampedNode::publishTimerCallback, this));
  publishActiveName("idle");
}

void CmdVelMuxStampedNode::configureInputs(const std::vector<std::string> & input_names)
{
  if (input_names.empty()) {
    throw std::runtime_error("input_names must contain at least one cmd_vel source");
  }

  inputs_.clear();
  inputs_.reserve(input_names.size());
  for (const auto & name : input_names) {
    if (name.empty()) {
      throw std::runtime_error("input_names must not contain an empty name");
    }

    StampedMuxInputState input;
    input.name = name;
    input.topic = declare_parameter<std::string>(
      "inputs." + name + ".topic",
      defaultTopicForInput(name));
    input.timeout = declare_parameter<double>(
      "inputs." + name + ".timeout",
      defaultTimeoutForInput(name));
    input.priority = declare_parameter<int>(
      "inputs." + name + ".priority",
      defaultPriorityForInput(name));
    if (input.timeout <= 0.0) {
      throw std::runtime_error("inputs." + name + ".timeout must be positive");
    }
    inputs_.push_back(std::move(input));
  }

  for (std::size_t index = 0; index < inputs_.size(); ++index) {
    inputs_[index].subscription = create_subscription<geometry_msgs::msg::TwistStamped>(
      inputs_[index].topic,
      10,
      [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        inputCallback(index, msg);
      });
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel source '%s' on '%s' priority=%d timeout=%.3fs",
      inputs_[index].name.c_str(),
      inputs_[index].topic.c_str(),
      inputs_[index].priority,
      inputs_[index].timeout);
  }
}

void CmdVelMuxStampedNode::inputCallback(
  std::size_t input_index,
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (input_index >= inputs_.size()) {
    return;
  }
  inputs_[input_index].last_msg = *msg;
  inputs_[input_index].last_time = get_clock()->now();
}

void CmdVelMuxStampedNode::publishTimerCallback()
{
  const auto now = get_clock()->now();
  const auto * selected = selectActiveStampedInput(inputs_, now);
  publishActiveName(selected == nullptr ? "idle" : selected->name);
  if (selected == nullptr) {
    return;
  }
  output_pub_->publish(makeMuxOutput(*selected));
}

void CmdVelMuxStampedNode::publishActiveName(const std::string & active_name)
{
  if (active_name_ == active_name) {
    return;
  }
  active_name_ = active_name;
  std_msgs::msg::String msg;
  msg.data = active_name_;
  active_pub_->publish(msg);
}

}  // namespace hanmole_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelMuxStampedNode)
