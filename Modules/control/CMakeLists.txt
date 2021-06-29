cmake_minimum_required(VERSION 2.8.3)
project(prometheus_control)

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
)


###############################
##      声明可执行cpp文件     ##
###############################

add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)

###### Main File ##########
##rover_pos_controller.cpp
add_executable(rover_pos_controller src/rover_control/rover_pos_controller.cpp)
add_dependencies(rover_pos_controller prometheus_control_gencpp) 
target_link_libraries(rover_pos_controller ${catkin_LIBRARIES})

##px4_pos_controller.cpp
add_executable(px4_pos_controller src/px4_pos_controller.cpp)
add_dependencies(px4_pos_controller prometheus_control_gencpp) 
target_link_libraries(px4_pos_controller ${catkin_LIBRARIES})

##px4_geometry_controller.cpp
add_executable(px4_geometry_controller src/px4_geometry_controller.cpp)
add_dependencies(px4_geometry_controller prometheus_control_gencpp) 
target_link_libraries(px4_geometry_controller ${catkin_LIBRARIES})

##px4_pos_estimator.cpp
add_executable(px4_pos_estimator src/px4_pos_estimator.cpp)
add_dependencies(px4_pos_estimator prometheus_control_gencpp)
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator OptiTrackFeedbackRigidBody)

##ros_info_color_test.cpp
add_executable(ros_info_color_test src/Utilities/ros_info_color_test.cpp)
add_dependencies(ros_info_color_test prometheus_control_gencpp)
target_link_libraries(ros_info_color_test ${catkin_LIBRARIES})

# add_executable(planner_server src/Test/planner_server.cpp)
# add_dependencies(planner_server prometheus_control_gencpp)
# target_link_libraries(planner_server ${catkin_LIBRARIES})


###### Test File ##########
#add_executable(px4_fw_controller src/Test/px4_fw_controller.cpp)
#add_dependencies(px4_fw_controller prometheus_control_gencpp)
#target_link_libraries(px4_fw_controller ${catkin_LIBRARIES})

add_executable(px4_sender src/px4_sender.cpp)
add_dependencies(px4_sender prometheus_control_gencpp)
target_link_libraries(px4_sender ${catkin_LIBRARIES})


###### Utilities File ##########
#add_executable(keyboard_control src/Utilities/keyboard_control.cpp)
#add_dependencies(keyboard_control prometheus_control_gencpp)
#target_link_libraries(keyboard_control ${catkin_LIBRARIES})
#target_link_libraries(keyboard_control KeyboardEvent)

##px4_pos_estimator_with_noise.cpp
add_executable(px4_pos_estimator_with_noise src//Utilities/px4_pos_estimator_with_noise.cpp)
add_dependencies(px4_pos_estimator_with_noise prometheus_control_gencpp)
target_link_libraries(px4_pos_estimator_with_noise ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator_with_noise OptiTrackFeedbackRigidBody)

add_executable(filter_tester src/Utilities/filter_tester.cpp)
add_dependencies(filter_tester prometheus_control_gencpp)
target_link_libraries(filter_tester ${catkin_LIBRARIES})

add_executable(TFmini src/Utilities/TFmini.cpp)
add_dependencies(TFmini prometheus_control_gencpp)
target_link_libraries(TFmini ${catkin_LIBRARIES})

add_executable(fake_vicon src/Utilities/fake_vicon.cpp)
add_dependencies(fake_vicon prometheus_control_gencpp)
target_link_libraries(fake_vicon ${catkin_LIBRARIES})

add_executable(terminal_control src/Utilities/terminal_control.cpp)
add_dependencies(terminal_control prometheus_control_gencpp)
target_link_libraries(terminal_control ${catkin_LIBRARIES})
target_link_libraries(terminal_control KeyboardEvent)

add_executable(terminal_control_rover src/Utilities/terminal_control_rover.cpp)
add_dependencies(terminal_control_rover prometheus_control_gencpp)
target_link_libraries(terminal_control_rover ${catkin_LIBRARIES})

add_executable(OptiTrackTest src/Utilities/OptiTrackTest.cpp)
add_dependencies(OptiTrackTest prometheus_control_gencpp)
target_link_libraries(OptiTrackTest ${catkin_LIBRARIES})
target_link_libraries(OptiTrackTest OptiTrackFeedbackRigidBody)
target_link_libraries(OptiTrackTest KeyboardEvent)

add_executable(set_mode src/Utilities/set_mode.cpp)
add_dependencies(set_mode prometheus_control_gencpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})

add_executable(eigen_test src/Utilities/eigen_test.cpp)
add_dependencies(eigen_test prometheus_control_gencpp)
target_link_libraries(eigen_test ${catkin_LIBRARIES})
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
