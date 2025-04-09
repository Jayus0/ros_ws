cmake_minimum_required(VERSION 3.0.2)
project(topic_commu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_commu
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub_node src/pub_node.cpp)
target_link_libraries(pub_node
  ${catkin_LIBRARIES}
)


add_executable(sub_node src/sub_node.cpp)
target_link_libraries(sub_node
  ${catkin_LIBRARIES}
)
