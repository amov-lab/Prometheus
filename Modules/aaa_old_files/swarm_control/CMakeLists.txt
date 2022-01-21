cmake_minimum_required(VERSION 2.8.3)
project(prometheus_swarm_control)

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../common/include
  ../ego_planner/ego-planner-swarm/src/uav_simulator/Utils/uav_utils/include  
)


###############################
##      声明可执行cpp文件     ##
###############################


##
add_executable(swarm_estimator src/swarm_estimator.cpp)
add_dependencies(swarm_estimator prometheus_swarm_control_gencpp)
target_link_libraries(swarm_estimator ${catkin_LIBRARIES})

add_executable(swarm_controller src/swarm_controller.cpp)
add_dependencies(swarm_controller prometheus_swarm_control_gencpp)
target_link_libraries(swarm_controller ${catkin_LIBRARIES})

add_executable(swarm_ground_station src/swarm_ground_station.cpp)
add_dependencies(swarm_ground_station prometheus_swarm_control_gencpp)
target_link_libraries(swarm_ground_station ${catkin_LIBRARIES})

add_executable(swarm_formation_control src/swarm_formation_control.cpp)
add_dependencies(swarm_formation_control prometheus_swarm_control_gencpp)
target_link_libraries(swarm_formation_control ${catkin_LIBRARIES})

add_executable(swarm_terminal_control src/swarm_terminal_control.cpp)
add_dependencies(swarm_terminal_control prometheus_swarm_control_gencpp)
target_link_libraries(swarm_terminal_control ${catkin_LIBRARIES})

add_executable(ego_traj_to_cmd src/ego_traj_to_cmd.cpp)
add_dependencies(ego_traj_to_cmd prometheus_swarm_control_gencpp)
target_link_libraries(ego_traj_to_cmd ${catkin_LIBRARIES})
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_prometheus_swarm_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
