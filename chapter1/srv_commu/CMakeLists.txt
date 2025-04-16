cmake_minimum_required(VERSION 3.0.2)
project(srv_commu)

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
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES srv_commu
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(server_node src/server_node.cpp)

target_link_libraries(server_node
  ${catkin_LIBRARIES}
)

add_executable(client_node src/client_node.cpp)

target_link_libraries(client_node
  ${catkin_LIBRARIES}
)