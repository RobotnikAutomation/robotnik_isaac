// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(IsaacSystem, loads_robot_agnostic_joint_and_interfaces)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  const std::string urdf = R"(
    <robot name="generic_test_robot">
      <link name="base"/><link name="steering"/>
      <joint name="project_specific_steering" type="continuous">
        <parent link="base"/><child link="steering"/>
      </joint>
      <ros2_control name="IsaacTestSystem" type="system">
        <hardware>
          <plugin>robotnik_isaac_ros2_control/IsaacSystem</plugin>
          <param name="state_topic">state</param>
          <param name="command_topic">command</param>
        </hardware>
        <joint name="project_specific_steering">
          <command_interface name="effort">
            <param name="min">-2.0</param><param name="max">2.0</param>
            <param name="initial_value">0.0</param>
          </command_interface>
          <state_interface name="position"><param name="initial_value">1.25</param></state_interface>
          <state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>
        </joint>
      </ros2_control>
    </robot>)";

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  hardware_interface::ResourceManager resources(
    urdf, clock, rclcpp::get_logger("test_generic_system"), true);

  EXPECT_TRUE(resources.state_interface_exists("project_specific_steering/position"));
  EXPECT_TRUE(resources.state_interface_exists("project_specific_steering/velocity"));
  EXPECT_TRUE(resources.command_interface_exists("project_specific_steering/effort"));
  EXPECT_FALSE(resources.command_interface_exists("project_specific_steering/velocity"));
}
