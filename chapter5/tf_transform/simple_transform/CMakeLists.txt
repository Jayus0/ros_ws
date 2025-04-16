cmake_minimum_required(VERSION 3.0.2)
project(simple_transform)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_transform
#  CATKIN_DEPENDS roscpp rospy std_msgs tf tf2
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tf_broadcaster_node
  ${catkin_LIBRARIES}
)


add_executable(tf_listener_node src/tf_listener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tf_listener_node
  ${catkin_LIBRARIES}
)
