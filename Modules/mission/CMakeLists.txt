cmake_minimum_required(VERSION 2.8.3)
project(prometheus_mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
  prometheus_msgs
)

##  System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  sensor_msgs
  std_msgs
)

catkin_package(
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
  ../control/include
)


###############################
##      声明可执行cpp文件     ##
###############################
add_executable(pad_tracking test/pad_tracking.cpp)
add_dependencies(pad_tracking prometheus_mission_gencpp)
target_link_libraries(pad_tracking ${catkin_LIBRARIES})

add_executable(mission_cmd_pub test/mission_cmd_pub.cpp)
add_dependencies(mission_cmd_pub prometheus_mission_gencpp)
target_link_libraries(mission_cmd_pub ${catkin_LIBRARIES})

add_executable(turtlebot_formation formation_flight/turtlebot_formation.cpp)
add_dependencies(turtlebot_formation prometheus_mission_gencpp)
target_link_libraries(turtlebot_formation ${catkin_LIBRARIES})

add_executable(circle_crossing circle_crossing/circle_crossing.cpp)
add_dependencies(circle_crossing prometheus_mission_gencpp)
target_link_libraries(circle_crossing ${catkin_LIBRARIES})

add_executable(number_detection number_detection/number_detection.cpp)
add_dependencies(number_detection prometheus_mission_gencpp)
target_link_libraries(number_detection ${catkin_LIBRARIES})

add_executable(formation_flight formation_flight/formation_flight.cpp)
add_dependencies(formation_flight prometheus_mission_gencpp)
target_link_libraries(formation_flight ${catkin_LIBRARIES})

add_executable(color_line_following color_line_following/color_line_following.cpp)
add_dependencies(color_line_following prometheus_mission_gencpp)
target_link_libraries(color_line_following ${catkin_LIBRARIES})

add_executable(planning_mission planning_mission/planning_mission.cpp)
add_dependencies(planning_mission prometheus_mission_gencpp)
target_link_libraries(planning_mission ${catkin_LIBRARIES})

add_executable(pub_goal planning_mission/pub_goal.cpp)
add_dependencies(pub_goal prometheus_mission_gencpp)
target_link_libraries(pub_goal ${catkin_LIBRARIES})

add_executable(pub_goal_swarm planning_mission/pub_goal_swarm.cpp)
add_dependencies(pub_goal_swarm prometheus_mission_gencpp)
target_link_libraries(pub_goal_swarm ${catkin_LIBRARIES})

add_executable(waypoint_tracking waypoint_tracking/waypoint_tracking.cpp)
add_dependencies(waypoint_tracking prometheus_mission_gencpp)
target_link_libraries(waypoint_tracking ${catkin_LIBRARIES})

add_executable(autonomous_landing autonomous_landing/autonomous_landing.cpp)
add_dependencies(autonomous_landing prometheus_mission_gencpp)
target_link_libraries(autonomous_landing ${catkin_LIBRARIES})

add_executable(autonomous_landing_outdoor autonomous_landing/autonomous_landing_outdoor.cpp)
add_dependencies(autonomous_landing_outdoor prometheus_mission_gencpp)
target_link_libraries(autonomous_landing_outdoor ${catkin_LIBRARIES})


add_executable(object_tracking object_tracking/object_tracking.cpp)
add_dependencies(object_tracking prometheus_mission_gencpp)
target_link_libraries(object_tracking ${catkin_LIBRARIES})

add_executable(indoor_competition indoor_competition/indoor_competition.cpp)
add_dependencies(indoor_competition prometheus_mission_gencpp)
target_link_libraries(indoor_competition ${catkin_LIBRARIES})

add_executable(fake_detectioninfo test/fake_detectioninfo.cpp)
add_dependencies(fake_detectioninfo prometheus_mission_gencpp)
target_link_libraries(fake_detectioninfo ${catkin_LIBRARIES})

add_executable(formation_getpose formation/formation_getpose.cpp)
add_dependencies(formation_getpose prometheus_mission_gencpp)
target_link_libraries(formation_getpose ${catkin_LIBRARIES})

add_executable(formation_state formation/formation_state.cpp)
add_dependencies(formation_state prometheus_mission_gencpp)
target_link_libraries(formation_state ${catkin_LIBRARIES})

add_executable(formation_move formation/formation_move.cpp)
add_dependencies(formation_move prometheus_mission_gencpp)
target_link_libraries(formation_move ${catkin_LIBRARIES})

add_executable(formation_change formation/formation_change.cpp)
add_dependencies(formation_change prometheus_mission_gencpp)
target_link_libraries(formation_change ${catkin_LIBRARIES})

add_executable(formation_setmode formation/formation_setmode.cpp)
add_dependencies(formation_setmode prometheus_mission_gencpp)
target_link_libraries(formation_setmode ${catkin_LIBRARIES})

add_executable(formation_control formation/formation_control.cpp)
add_dependencies(formation_control prometheus_mission_gencpp)
target_link_libraries(formation_control ${catkin_LIBRARIES})

add_executable(formation_square formation/formation_square.cpp)
add_dependencies(formation_square prometheus_mission_gencpp)
target_link_libraries(formation_square ${catkin_LIBRARIES})

add_executable(pub_goal_from_qgc planning_mission/pub_goal_from_qgc.cpp)
add_dependencies(pub_goal_from_qgc prometheus_mission_gencpp)
target_link_libraries(pub_goal_from_qgc ${catkin_LIBRARIES})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_prometheus_mission.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
