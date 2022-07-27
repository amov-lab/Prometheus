cmake_minimum_required(VERSION 3.0.2)
project(prometheus_communication_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  message_generation
  tf2_msgs
  visualization_msgs
  mavros
  mavros_msgs
  prometheus_msgs
)


generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2_msgs
  visualization_msgs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  ##CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE CURRENT_INCLUDE include/*.hpp include/*.h)
file(GLOB_RECURSE CURRENT_SOURCE src/*.cpp)

## Specify libraries to link a library or executable target against
add_executable(communication_bridge ${CURRENT_SOURCE} ${CURRENT_INCLUDE})
add_dependencies(communication_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(communication_bridge ${catkin_LIBRARIES} -lboost_serialization)

