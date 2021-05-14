cmake_minimum_required(VERSION 2.8.3)
project(prometheus_ego)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_eigen
  prometheus_msgs
  quadrotor_msgs
  geometry_msgs
  mavros_msgs
  uav_utils
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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../../common/include
  ../../ground_station/include
)


###############################
##      声明可执行cpp文件     ##
###############################

# add_executable(prometheus_ego src/prometheus_ego.cpp)
# add_dependencies(prometheus_ego prometheus_ego_gencpp)
# target_link_libraries(prometheus_ego ${catkin_LIBRARIES})

add_executable(px4_ego_estimator src/px4_ego_estimator.cpp)
add_dependencies(px4_ego_estimator prometheus_ego_gencpp)
target_link_libraries(px4_ego_estimator ${catkin_LIBRARIES})

add_executable(px4_ego_controller src/px4_ego_controller.cpp)
add_dependencies(px4_ego_controller prometheus_ego_gencpp)
target_link_libraries(px4_ego_controller ${catkin_LIBRARIES})

add_executable(ground_station src/ground_station.cpp)
add_dependencies(ground_station prometheus_ego_gencpp)
target_link_libraries(ground_station ${catkin_LIBRARIES})

add_executable(px4_ego_terminal_control src/px4_ego_terminal_control.cpp)
add_dependencies(px4_ego_terminal_control prometheus_ego_gencpp)
target_link_libraries(px4_ego_terminal_control ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_prometheus_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
