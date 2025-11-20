cmake_minimum_required(VERSION 3.8)
project(marm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch 和 urdf 目录
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 RViz 配置文件
install(FILES urdf_ros2.rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()