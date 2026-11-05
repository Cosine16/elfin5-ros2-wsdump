cmake_minimum_required(VERSION 3.5)
project(cos_rsvisual)

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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED)

# 五个组件编译为同一个 SHARED 库
add_library(cos_rsvisual_components SHARED
  src/person_depth_detector.cpp
  src/person_visual_detector.cpp
  src/face_visual_detector.cpp
  src/arm_follower.cpp
  src/obstacle_updater.cpp
)
ament_target_dependencies(cos_rsvisual_components
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
target_link_libraries(cos_rsvisual_components ${OpenCV_LIBS})

# foxy 写法:每个 rclcpp_components_register_nodes 只注册一个类
rclcpp_components_register_nodes(cos_rsvisual_components "cos_rsvisual::PersonDepthDetector")
rclcpp_components_register_nodes(cos_rsvisual_components "cos_rsvisual::PersonVisualDetector")
rclcpp_components_register_nodes(cos_rsvisual_components "cos_rsvisual::FaceVisualDetector")
rclcpp_components_register_nodes(cos_rsvisual_components "cos_rsvisual::ArmFollower")
rclcpp_components_register_nodes(cos_rsvisual_components "cos_rsvisual::ObstacleUpdater")

install(TARGETS cos_rsvisual_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch urdf config rviz worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
