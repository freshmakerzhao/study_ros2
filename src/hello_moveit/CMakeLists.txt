cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# ==========================================================
# 1. hello_moveit 配置
# ==========================================================
add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

# ==========================================================
# 2. visualizing_in_rviz 配置 
# ==========================================================
add_executable(visualizing_in_rviz src/visualizing_in_rviz.cpp)

target_include_directories(visualizing_in_rviz PUBLIC        # 设置头文件
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(visualizing_in_rviz PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(                                # 链接依赖库
  visualizing_in_rviz
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# ==========================================================
# 安装设置
# ==========================================================
install(TARGETS 
  hello_moveit 
  visualizing_in_rviz
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()