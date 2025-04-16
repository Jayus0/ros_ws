cmake_minimum_required(VERSION 3.0.2)
project(time_sync)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  common_msg
  message_filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES time_sync
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(time_sync_test src/test.cpp)
target_link_libraries(time_sync_test
  ${catkin_LIBRARIES}
)


add_executable(time_sync src/time_sync.cpp)
target_link_libraries(time_sync
  ${catkin_LIBRARIES}
)


