// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#include "robotnik_isaac_ros2_control/isaac_system.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace robotnik_isaac_ros2_control
{

const std::array<std::string, IsaacSystem::kWheelCount> IsaacSystem::wheel_names_ = {
  "robot_front_left_wheel_joint",
  "robot_front_right_wheel_joint",
  "robot_back_left_wheel_joint",
  "robot_back_right_wheel_joint",
};

namespace
{

bool has_exact_interfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::unordered_set<std::string> & expected)
{
  if (interfaces.size() != expected.size()) {
    return false;
  }

  std::unordered_set<std::string> actual;
  for (const auto & interface : interfaces) {
    actual.insert(interface.name);
  }
  return actual == expected;
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

  const auto & hardware_info = get_hardware_info();
  if (hardware_info.joints.size() != kWheelCount) {
    RCLCPP_ERROR(
      get_logger(), "IsaacSystem requires exactly %zu wheel joints, but the URDF declares %zu.",
      kWheelCount, hardware_info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::unordered_set<std::string> expected_command_interfaces = {
    hardware_interface::HW_IF_VELOCITY,
  };
  const std::unordered_set<std::string> expected_state_interfaces = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT,
  };

  for (const auto & wheel_name : wheel_names_) {
    const auto joint = std::find_if(
      hardware_info.joints.begin(), hardware_info.joints.end(),
      [&wheel_name](const auto & candidate) {return candidate.name == wheel_name;});
    if (joint == hardware_info.joints.end()) {
      RCLCPP_ERROR(get_logger(), "Required Isaac wheel joint '%s' is missing.", wheel_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_exact_interfaces(joint->command_interfaces, expected_command_interfaces)) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose only the velocity command interface.",
        wheel_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_exact_interfaces(joint->state_interfaces, expected_state_interfaces)) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose position, velocity and effort state interfaces.",
        wheel_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto state_topic = hardware_info.hardware_parameters.find("state_topic");
  if (state_topic != hardware_info.hardware_parameters.end() && !state_topic->second.empty()) {
    state_topic_ = state_topic->second;
  }
  const auto command_topic = hardware_info.hardware_parameters.find("command_topic");
  if (command_topic != hardware_info.hardware_parameters.end() && !command_topic->second.empty()) {
    command_topic_ = command_topic->second;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (std::size_t index = 0; index < wheel_names_.size(); ++index) {
    const auto & joint_name = wheel_names_[index];
    position_states_[index] = get_state_interface_handle(
      joint_name + "/" + hardware_interface::HW_IF_POSITION);
    velocity_states_[index] = get_state_interface_handle(
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    effort_states_[index] = get_state_interface_handle(
      joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    velocity_commands_[index] = get_command_interface_handle(
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  const auto node = get_node();
  state_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IsaacSystem::joint_state_callback, this, std::placeholders::_1));
  command_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, rclcpp::QoS(1));

  RCLCPP_INFO(
    get_logger(), "Configured Isaac joint bridge: '%s' -> ros2_control -> '%s'.",
    state_topic_.c_str(), command_topic_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void IsaacSystem::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t wheel_index = 0; wheel_index < wheel_names_.size(); ++wheel_index) {
    const auto found = std::find(
      message->name.begin(), message->name.end(), wheel_names_[wheel_index]);
    if (found == message->name.end()) {
      continue;
    }

    const auto message_index = static_cast<std::size_t>(
      std::distance(message->name.begin(), found));
    auto & state = latest_wheel_states_[wheel_index];
    if (message_index < message->position.size()) {
      state.position = message->position[message_index];
    }
    if (message_index < message->velocity.size()) {
      state.velocity = message->velocity[message_index];
    }
    if (message_index < message->effort.size()) {
      state.effort = message->effort[message_index];
    }
  }
}

hardware_interface::return_type IsaacSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::array<WheelState, kWheelCount> wheel_states;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    wheel_states = latest_wheel_states_;
  }

  for (std::size_t index = 0; index < wheel_names_.size(); ++index) {
    const auto & state = wheel_states[index];
    (void)position_states_[index]->set_value(state.position);
    (void)velocity_states_[index]->set_value(state.velocity);
    (void)effort_states_[index]->set_value(state.effort);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!command_publisher_) {
    return hardware_interface::return_type::ERROR;
  }

  sensor_msgs::msg::JointState command;
  command.header.stamp = time;
  command.name.assign(wheel_names_.begin(), wheel_names_.end());
  command.velocity.reserve(kWheelCount);
  for (const auto & velocity_command : velocity_commands_) {
    const auto value = velocity_command->get_optional<double>();
    command.velocity.push_back(value.value_or(0.0));
  }

  command_publisher_->publish(command);
  return hardware_interface::return_type::OK;
}

}  // namespace robotnik_isaac_ros2_control

PLUGINLIB_EXPORT_CLASS(
  robotnik_isaac_ros2_control::IsaacSystem,
  hardware_interface::SystemInterface)
