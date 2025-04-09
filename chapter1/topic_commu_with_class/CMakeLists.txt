cmake_minimum_required(VERSION 3.0.2)
project(topic_commu_with_class)

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

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES topic_commu_with_class
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pub_node_c src/pub_node_with_class.cpp)
target_link_libraries(pub_node_c
  ${catkin_LIBRARIES}
)

