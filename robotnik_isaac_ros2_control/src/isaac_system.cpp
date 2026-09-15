// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#include "robotnik_isaac_ros2_control/isaac_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace robotnik_isaac_ros2_control
{
namespace
{

constexpr std::size_t kPosition = 0;
constexpr std::size_t kVelocity = 1;
constexpr std::size_t kEffort = 2;
const std::array<std::string, 3> kInterfaces = {
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT,
};

bool parse_bool(const std::string & value)
{
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::invalid_argument("expected true, false, 1 or 0");
}

template<typename T>
void read_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, T & output);

template<>
void read_parameter<std::string>(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, std::string & output)
{
  const auto found = parameters.find(name);
  if (found != parameters.end() && !found->second.empty()) {
    output = found->second;
  }
}

template<>
void read_parameter<bool>(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, bool & output)
{
  const auto found = parameters.find(name);
  if (found != parameters.end()) {
    output = parse_bool(found->second);
  }
}

double finite_limit_or(const std::string & value, double fallback)
{
  if (value.empty()) {
    return fallback;
  }
  const double parsed = std::stod(value);
  return std::isfinite(parsed) ? parsed : fallback;
}

bool supports_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const auto & interface) {return interface.name == name;});
}

}  // namespace

hardware_interface::CallbackReturn IsaacSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto & hardware_info = get_hardware_info();
    const auto & parameters = hardware_info.hardware_parameters;
    read_parameter(parameters, "state_topic", state_topic_);
    read_parameter(parameters, "command_topic", command_topic_);
    // The unsuffixed parameter remains a convenient override for integrations
    // that intentionally use one profile in both directions.
    if (parameters.find("qos_reliability") != parameters.end()) {
      read_parameter(parameters, "qos_reliability", state_qos_reliability_);
      command_qos_reliability_ = state_qos_reliability_;
    }
    read_parameter(parameters, "state_qos_reliability", state_qos_reliability_);
    read_parameter(parameters, "command_qos_reliability", command_qos_reliability_);
    read_parameter(parameters, "require_all_joints", require_all_joints_);
    read_parameter(parameters, "reject_nonfinite_state", reject_nonfinite_state_);
    read_parameter(parameters, "enforce_command_limits", enforce_command_limits_);

    if (const auto found = parameters.find("qos_depth"); found != parameters.end()) {
      qos_depth_ = std::stoul(found->second);
      if (qos_depth_ == 0) {
        throw std::invalid_argument("qos_depth must be greater than zero");
      }
    }
    if (const auto found = parameters.find("state_timeout"); found != parameters.end()) {
      state_timeout_seconds_ = std::stod(found->second);
      if (!std::isfinite(state_timeout_seconds_) || state_timeout_seconds_ < 0.0) {
        throw std::invalid_argument("state_timeout must be finite and non-negative");
      }
    }
    const auto valid_reliability = [](const auto & value) {
        return value == "best_effort" || value == "reliable";
      };
    if (!valid_reliability(state_qos_reliability_) ||
      !valid_reliability(command_qos_reliability_))
    {
      throw std::invalid_argument("QoS reliability must be best_effort or reliable");
    }
    if (hardware_info.joints.empty()) {
      throw std::invalid_argument("at least one joint must be declared");
    }

    joints_.reserve(hardware_info.joints.size());
    for (const auto & source : hardware_info.joints) {
      if (!joint_index_.emplace(source.name, joints_.size()).second) {
        throw std::invalid_argument("duplicate joint '" + source.name + "'");
      }
      Joint joint;
      joint.name = source.name;
      bool known_interface = false;
      for (std::size_t index = 0; index < kInterfaces.size(); ++index) {
        if (supports_interface(source.state_interfaces, kInterfaces[index]) ||
          supports_interface(source.command_interfaces, kInterfaces[index]))
        {
          known_interface = true;
        }
      }
      for (const auto & interface : source.state_interfaces) {
        if (std::find(kInterfaces.begin(), kInterfaces.end(), interface.name) == kInterfaces.end()) {
          throw std::invalid_argument(
                  "unsupported state interface '" + interface.name + "' on joint '" + source.name + "'");
        }
      }
      for (const auto & interface : source.command_interfaces) {
        const auto found = std::find(kInterfaces.begin(), kInterfaces.end(), interface.name);
        if (found == kInterfaces.end()) {
          throw std::invalid_argument(
                  "unsupported command interface '" + interface.name + "' on joint '" + source.name + "'");
        }
        const auto index = static_cast<std::size_t>(std::distance(kInterfaces.begin(), found));
        joint.minimum[index] = finite_limit_or(interface.min, joint.minimum[index]);
        joint.maximum[index] = finite_limit_or(interface.max, joint.maximum[index]);
        if (joint.minimum[index] > joint.maximum[index]) {
          throw std::invalid_argument("invalid command limits on joint '" + source.name + "'");
        }
      }
      if (!known_interface) {
        throw std::invalid_argument("joint '" + source.name + "' has no supported interfaces");
      }
      joints_.push_back(std::move(joint));
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid IsaacSystem configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::on_configure(const rclcpp_lifecycle::State &)
{
  const auto & hardware_info = get_hardware_info();
  for (std::size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
    auto & joint = joints_[joint_index];
    const auto & source = hardware_info.joints[joint_index];
    for (std::size_t interface_index = 0; interface_index < kInterfaces.size(); ++interface_index) {
      const auto full_name = joint.name + "/" + kInterfaces[interface_index];
      if (supports_interface(source.state_interfaces, kInterfaces[interface_index])) {
        joint.state_handles[interface_index] = get_state_interface_handle(full_name);
        if (const auto initial = joint.state_handles[interface_index]->get_optional<double>()) {
          joint.state[interface_index] = *initial;
        }
      }
      if (supports_interface(source.command_interfaces, kInterfaces[interface_index])) {
        joint.command_handles[interface_index] = get_command_interface_handle(full_name);
      }
    }
  }

  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_));
  if (state_qos_reliability_ == "reliable") {
    state_qos.reliable();
  } else {
    state_qos.best_effort();
  }
  auto command_qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_));
  if (command_qos_reliability_ == "reliable") {
    command_qos.reliable();
  } else {
    command_qos.best_effort();
  }
  const auto node = get_node();
  state_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, state_qos,
    std::bind(&IsaacSystem::joint_state_callback, this, std::placeholders::_1));
  command_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, command_qos);

  RCLCPP_INFO(
    get_logger(), "Configured generic Isaac bridge for %zu joints: '%s' -> '%s'.",
    joints_.size(), state_topic_.c_str(), command_topic_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::stop_transport()
{
  active_ = false;
  state_subscriber_.reset();
  command_publisher_.reset();
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_received_ = false;
  for (auto & joint : joints_) {
    joint.received.fill(false);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  return stop_transport();
}

hardware_interface::CallbackReturn IsaacSystem::on_error(const rclcpp_lifecycle::State &)
{
  return stop_transport();
}

void IsaacSystem::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::unordered_set<std::string> seen;
  std::lock_guard<std::mutex> lock(state_mutex_);
  bool accepted_value = false;
  for (std::size_t message_index = 0; message_index < message->name.size(); ++message_index) {
    const auto & name = message->name[message_index];
    const auto found = joint_index_.find(name);
    if (found == joint_index_.end()) {
      continue;
    }
    if (!seen.insert(name).second) {
      RCLCPP_WARN(get_logger(), "Ignoring duplicate joint '%s' in JointState.", name.c_str());
      continue;
    }
    auto & joint = joints_[found->second];
    const std::array<const std::vector<double> *, 3> values = {
      &message->position, &message->velocity, &message->effort};
    for (std::size_t interface_index = 0; interface_index < values.size(); ++interface_index) {
      if (message_index >= values[interface_index]->size()) {
        continue;
      }
      const double value = (*values[interface_index])[message_index];
      if (!std::isfinite(value) && reject_nonfinite_state_) {
        continue;
      }
      joint.state[interface_index] = value;
      joint.received[interface_index] = true;
      accepted_value = true;
    }
  }
  if (accepted_value) {
    state_received_ = true;
    last_state_time_ = get_node()->now();
  }
}

hardware_interface::return_type IsaacSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_timeout_seconds_ > 0.0 && state_received_ &&
    (time - last_state_time_).seconds() > state_timeout_seconds_)
  {
    RCLCPP_ERROR(get_logger(), "Isaac joint state watchdog expired after %.3f s.", state_timeout_seconds_);
    return hardware_interface::return_type::ERROR;
  }
  if (require_all_joints_) {
    for (const auto & joint : joints_) {
      for (std::size_t index = 0; index < kInterfaces.size(); ++index) {
        if (joint.state_handles[index] && !joint.received[index]) {
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }
  for (auto & joint : joints_) {
    for (std::size_t index = 0; index < kInterfaces.size(); ++index) {
      if (joint.state_handles[index]) {
        (void)joint.state_handles[index]->set_value(joint.state[index]);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!active_ || !command_publisher_) {
    return hardware_interface::return_type::ERROR;
  }

  sensor_msgs::msg::JointState command;
  command.header.stamp = time;
  for (const auto & joint : joints_) {
    command.name.push_back(joint.name);
  }
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  std::array<std::vector<double> *, 3> output = {
    &command.position, &command.velocity, &command.effort};
  for (std::size_t interface_index = 0; interface_index < kInterfaces.size(); ++interface_index) {
    const bool any = std::any_of(
      joints_.begin(), joints_.end(),
      [interface_index](const auto & joint) {return static_cast<bool>(joint.command_handles[interface_index]);});
    if (!any) {
      continue;
    }
    output[interface_index]->reserve(joints_.size());
    for (const auto & joint : joints_) {
      double value = unavailable;
      if (joint.command_handles[interface_index]) {
        const auto requested = joint.command_handles[interface_index]->get_optional<double>();
        if (requested && std::isfinite(*requested)) {
          value = enforce_command_limits_ ?
            std::clamp(*requested, joint.minimum[interface_index], joint.maximum[interface_index]) :
            *requested;
        }
      }
      output[interface_index]->push_back(value);
    }
  }
  command_publisher_->publish(command);
  return hardware_interface::return_type::OK;
}

}  // namespace robotnik_isaac_ros2_control

PLUGINLIB_EXPORT_CLASS(
  robotnik_isaac_ros2_control::IsaacSystem,
  hardware_interface::SystemInterface)
