cmake_minimum_required(VERSION 2.8.3)
project(marm_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  urdf
  xacro
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
