// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#ifndef ROBOTNIK_ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_
#define ROBOTNIK_ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

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
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    std::array<double, 3> state{0.0, 0.0, 0.0};
    std::array<bool, 3> received{false, false, false};
    std::array<hardware_interface::StateInterface::SharedPtr, 3> state_handles{};
    std::array<hardware_interface::CommandInterface::SharedPtr, 3> command_handles{};
    std::array<double, 3> minimum{
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
    std::array<double, 3> maximum{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
  };

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  hardware_interface::CallbackReturn stop_transport();

  std::string state_topic_{"isaac/joint_states"};
  std::string command_topic_{"isaac/joint_command"};
  std::string state_qos_reliability_{"best_effort"};
  std::string command_qos_reliability_{"reliable"};
  std::size_t qos_depth_{5};
  double state_timeout_seconds_{0.0};
  bool require_all_joints_{false};
  bool reject_nonfinite_state_{true};
  bool enforce_command_limits_{true};
  bool active_{false};
  bool state_received_{false};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  std::vector<Joint> joints_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::mutex state_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
};

}  // namespace robotnik_isaac_ros2_control

#endif  // ROBOTNIK_ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_
