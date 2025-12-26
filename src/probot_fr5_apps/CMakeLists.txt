cmake_minimum_required(VERSION 3.8)
project(probot_fr5_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_pose_test_node
  src/moveit_pose_test_node.cpp
)

ament_target_dependencies(
  moveit_pose_test_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS moveit_pose_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
