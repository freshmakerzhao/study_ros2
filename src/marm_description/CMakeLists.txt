cmake_minimum_required(VERSION 3.8)
project(marm_description)

find_package(ament_cmake REQUIRED)

# 安装 urdf 和 launch 目录
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 RViz 配置文件
install(
  FILES urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
