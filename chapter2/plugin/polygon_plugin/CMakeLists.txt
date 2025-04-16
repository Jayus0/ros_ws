cmake_minimum_required(VERSION 3.0.2)
project(polygon_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  message_filters
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES polygon_plugin_lab
#  CATKIN_DEPENDS pluginlib roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_library(polygon_plugin_lab SHARED
  src/polyPlugin.cpp
)
target_link_libraries(polygon_plugin_lab
  ${catkin_LIBRARIES}
)

add_executable(polygon_lab src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(polygon_lab
  ${catkin_LIBRARIES}
)


