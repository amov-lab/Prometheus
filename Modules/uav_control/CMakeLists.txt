cmake_minimum_required(VERSION 2.8.3)
project(prometheus_uav_control)

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
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/Position_Controller
  ../common/include
)


###############################
##      声明可执行cpp文件     ##
###############################
add_library(uav_controller src/uav_controller.cpp)
add_library(uav_estimator src/uav_estimator.cpp)
target_link_libraries(uav_estimator ${catkin_LIBRARIES})

add_executable(uav_control_main src/uav_control_node.cpp)
add_dependencies(uav_control_main prometheus_uav_control_gencpp)
target_link_libraries(uav_control_main ${catkin_LIBRARIES})
target_link_libraries(uav_control_main uav_controller)
target_link_libraries(uav_control_main uav_estimator)

add_executable(uav_command_pub src/uav_command_pub.cpp)
add_dependencies(uav_command_pub prometheus_uav_control_gencpp)
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