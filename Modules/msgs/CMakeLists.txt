cmake_minimum_required(VERSION 2.8.3)
project(prometheus_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  DIRECTORY msg
  FILES
  DetectionInfo.msg
  ControlCommand.msg
  TrajectoryPoint.msg
  Trajectory.msg
  AttitudeReference.msg
  DroneState.msg
  Topic_for_log.msg
  ControlOutput.msg
  PlanningPositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
