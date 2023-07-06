cmake_minimum_required(VERSION 2.8.3)
project(prometheus_uav_control_fmt)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_eigen
  mavros_msgs
  prometheus_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs
  prometheus_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../common/include
)


###############################
##      声明可执行cpp文件     ##
###############################

add_executable(fmt_msg_receiver src/fmt_msg_receiver.cpp)
add_dependencies(fmt_msg_receiver prometheus_uav_control_fmt_gencpp)
target_link_libraries(fmt_msg_receiver ${catkin_LIBRARIES})

add_executable(fmt_test src/fmt_test.cpp)
add_dependencies(fmt_test prometheus_uav_control_fmt_gencpp)
target_link_libraries(fmt_test ${catkin_LIBRARIES})

add_executable(uav_command_pub src/uav_command_pub.cpp)
add_dependencies(uav_command_pub prometheus_uav_control_fmt_gencpp)
target_link_libraries(uav_command_pub ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)