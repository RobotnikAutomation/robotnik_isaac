cmake_minimum_required(VERSION 3.8)
project(robotnik_isaac_grippers)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch runtime urdf DESTINATION share/${PROJECT_NAME}
  PATTERN __pycache__ EXCLUDE
)
ament_package()
