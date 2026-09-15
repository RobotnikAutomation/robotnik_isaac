cmake_minimum_required(VERSION 3.8)
project(robotnik_isaac_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model_contract test/test_model_contract.py)
endif()

install(DIRECTORY assets config scripts urdf_templates
  DESTINATION share/${PROJECT_NAME}
  PATTERN __pycache__ EXCLUDE
)

ament_package()
