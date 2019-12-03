cmake_minimum_required(VERSION 2.8.3)
project(px4_command)

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
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  ControlCommand.msg
  TrajectoryPoint.msg
  Trajectory.msg
  AttitudeReference.msg
  DroneState.msg
  Topic_for_log.msg
  ControlOutput.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
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
)


###############################
##      声明可执行cpp文件     ##
###############################

add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)

###### Main File ##########

##px4_pos_controller.cpp
add_executable(px4_pos_controller src/px4_pos_controller.cpp)
add_dependencies(px4_pos_controller px4_command_gencpp) 
target_link_libraries(px4_pos_controller ${catkin_LIBRARIES})

##px4_pos_estimator.cpp
add_executable(px4_pos_estimator src/px4_pos_estimator.cpp)
add_dependencies(px4_pos_estimator px4_command_gencpp)
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator OptiTrackFeedbackRigidBody)

##px4_sender.cpp
add_executable(px4_sender src/px4_sender.cpp)
add_dependencies(px4_sender px4_command_gencpp)
target_link_libraries(px4_sender ${catkin_LIBRARIES})

##ground_station.cpp
add_executable(ground_station src/ground_station.cpp)
add_dependencies(ground_station px4_command_gencpp)
target_link_libraries(ground_station ${catkin_LIBRARIES})
target_link_libraries(ground_station OptiTrackFeedbackRigidBody)

###### Test File ##########
add_executable(px4_fw_controller src/Test/px4_fw_controller.cpp)
add_dependencies(px4_fw_controller px4_command_gencpp)
target_link_libraries(px4_fw_controller ${catkin_LIBRARIES})
###### Utilities File ##########

add_executable(setpoint_track src/Utilities/setpoint_track.cpp)
add_dependencies(setpoint_track px4_command_gencpp)
target_link_libraries(setpoint_track ${catkin_LIBRARIES})

add_executable(filter_tester src/Utilities/filter_tester.cpp)
add_dependencies(filter_tester px4_command_gencpp)
target_link_libraries(filter_tester ${catkin_LIBRARIES})

add_executable(TFmini src/Utilities/TFmini.cpp)
add_dependencies(TFmini px4_command_gencpp)
target_link_libraries(TFmini ${catkin_LIBRARIES})

add_executable(fake_vicon src/Utilities/fake_vicon.cpp)
add_dependencies(fake_vicon px4_command_gencpp)
target_link_libraries(fake_vicon ${catkin_LIBRARIES})

add_executable(move src/Utilities/move.cpp)
add_dependencies(move px4_command_gencpp)
target_link_libraries(move ${catkin_LIBRARIES})

add_executable(OptiTrackTest src/Utilities/OptiTrackTest.cpp)
add_dependencies(OptiTrackTest px4_command_gencpp)
target_link_libraries(OptiTrackTest ${catkin_LIBRARIES})
target_link_libraries(OptiTrackTest OptiTrackFeedbackRigidBody)
target_link_libraries(OptiTrackTest KeyboardEvent)

add_executable(set_mode src/Utilities/set_mode.cpp)
add_dependencies(set_mode px4_command_gencpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})

add_executable(eigen_test src/Utilities/eigen_test.cpp)
add_dependencies(eigen_test px4_command_gencpp)
target_link_libraries(eigen_test ${catkin_LIBRARIES})

###### Application File ##########
add_executable(square src/Application/square.cpp)
add_dependencies(square px4_command_gencpp)
target_link_libraries(square ${catkin_LIBRARIES})

add_executable(autonomous_landing src/Application/autonomous_landing.cpp)
add_dependencies(autonomous_landing px4_command_gencpp)
target_link_libraries(autonomous_landing ${catkin_LIBRARIES})

add_executable(target_tracking src/Application/target_tracking.cpp)
add_dependencies(target_tracking px4_command_gencpp)
target_link_libraries(target_tracking ${catkin_LIBRARIES})

add_executable(collision_avoidance src/Application/collision_avoidance.cpp)
add_dependencies(collision_avoidance px4_command_gencpp)
target_link_libraries(collision_avoidance ${catkin_LIBRARIES})

add_executable(collision_avoidance_streo src/Application/collision_avoidance_streo.cpp)
add_dependencies(collision_avoidance_streo px4_command_gencpp)
target_link_libraries(collision_avoidance_streo ${catkin_LIBRARIES})

add_executable(payload_drop src/Application/payload_drop.cpp)
add_dependencies(payload_drop px4_command_gencpp)
target_link_libraries(payload_drop ${catkin_LIBRARIES})

add_executable(formation_control_sitl src/Application/formation_control_sitl.cpp)
add_dependencies(formation_control_sitl px4_command_gencpp)
target_link_libraries(formation_control_sitl ${catkin_LIBRARIES})


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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_px4_command.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
