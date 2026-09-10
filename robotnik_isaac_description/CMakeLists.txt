cmake_minimum_required(VERSION 3.8)
project(robotnik_isaac_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY assets config scripts urdf_templates
  DESTINATION share/${PROJECT_NAME}
  PATTERN __pycache__ EXCLUDE
)

ament_package()
