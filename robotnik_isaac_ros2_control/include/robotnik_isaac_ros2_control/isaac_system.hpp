// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#ifndef ROBOTNIK_ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_
#define ROBOTNIK_ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_

#include <array>
#include <cstddef>
#include <mutex>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robotnik_isaac_ros2_control
{

class IsaacSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kWheelCount = 4;

  struct WheelState
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);

  static const std::array<std::string, kWheelCount> wheel_names_;

  std::string state_topic_{"isaac/joint_states"};
  std::string command_topic_{"isaac/joint_command"};

  std::array<WheelState, kWheelCount> latest_wheel_states_{};
  std::mutex state_mutex_;

  std::array<hardware_interface::StateInterface::SharedPtr, kWheelCount> position_states_;
  std::array<hardware_interface::StateInterface::SharedPtr, kWheelCount> velocity_states_;
  std::array<hardware_interface::StateInterface::SharedPtr, kWheelCount> effort_states_;
  std::array<hardware_interface::CommandInterface::SharedPtr, kWheelCount> velocity_commands_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
};

}  // namespace robotnik_isaac_ros2_control

#endif  // ROBOTNIK_ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_
