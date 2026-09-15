cmake_minimum_required(VERSION 3.8)
project(robotnik_isaac_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/isaac_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
)

pluginlib_export_plugin_description_file(
  hardware_interface robotnik_isaac_ros2_control_plugins.xml
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(realtime_tools REQUIRED)
  if(realtime_tools_VERSION VERSION_LESS "3.12.0")
    message(WARNING
      "Skipping ResourceManager test: hardware_interface 4.48 requires "
      "realtime_tools >= 3.12.0 (found ${realtime_tools_VERSION}).")
  else()
    ament_add_gtest(test_generic_system test/test_generic_system.cpp)
    target_link_libraries(test_generic_system ${PROJECT_NAME})
    ament_target_dependencies(test_generic_system hardware_interface rclcpp realtime_tools)
  endif()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES robotnik_isaac_ros2_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp sensor_msgs)
ament_package()
