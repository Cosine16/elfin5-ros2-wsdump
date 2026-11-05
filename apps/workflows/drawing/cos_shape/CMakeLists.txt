cmake_minimum_required(VERSION 3.5)
project(cos_shape)

# Export compile_commands.json so that clangd / VSCode IntelliSense
# can locate all include paths (ROS2 headers) from the build database.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14 (MoveIt2 Foxy requirement)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(circle_motion src/circle_motion_node.cpp)
ament_target_dependencies(circle_motion
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  trajectory_msgs
)

install(TARGETS circle_motion
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/circle_panel.py scripts/api_demo.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
