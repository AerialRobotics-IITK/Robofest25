cmake_minimum_required(VERSION 3.8)
project(human_tracking_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
#find_package(swarm_control REQUIRED)  # custom msg package

# All your custom headers live in src/
#include_directories(src/)

# 1. INCLUDE DIRECTORIES
# This tells CMake to look inside the 'include' folder for headers
include_directories(include)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()



add_executable(mavros_yaw_body_tracking
  src/mavros_yaw_body_tracking.cpp
)
ament_target_dependencies(mavros_yaw_body_tracking
  rclcpp
  std_msgs
  mavros_msgs
)

# Install the executables so `ros2 run` can find them
install(TARGETS

  mavros_yaw_body_tracking
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
