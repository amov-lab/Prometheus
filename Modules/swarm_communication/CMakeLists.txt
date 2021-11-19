cmake_minimum_required(VERSION 3.0.2)
project(prometheus_swarm_communication)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  prometheus_msgs
  tf2_eigen
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ../common/include
  ../swarm_control/include
)

## Specify libraries to link a library or executable target against
add_executable(cxy_msg_103_104 src/cxy_msg_103_104.cpp)
add_dependencies(cxy_msg_103_104 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cxy_msg_103_104 ${catkin_LIBRARIES})

add_executable(cxy_multicast_bridge src/cxy_multicast_bridge.cpp)
add_dependencies(cxy_multicast_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cxy_multicast_bridge ${catkin_LIBRARIES})

