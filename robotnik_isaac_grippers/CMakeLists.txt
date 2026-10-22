cmake_minimum_required(VERSION 3.8)
project(robotnik_isaac_grippers)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch runtime urdf DESTINATION share/${PROJECT_NAME}
  PATTERN __pycache__ EXCLUDE
)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_contract test/test_control_contract.py)
endif()
ament_package()
