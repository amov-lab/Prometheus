cmake_minimum_required(VERSION 2.8.3)
project(prometheus_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  actionlib_msgs
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
  UAVState.msg
  MultiUAVState.msg
  UAVCommand.msg
  UAVControlState.msg
  UAVSetup.msg
  TextInfo.msg
  GlobalAruco.msg
  UgvState.msg
  UgvCommand.msg
  MultiUGVState.msg
  

  ArucoInfo.msg
  MultiArucoInfo.msg
  DetectionInfo.msg
  MultiDetectionInfo.msg
  BoundingBox.msg
  BoundingBoxes.msg

  SwarmCommand.msg
  FormationAssign.msg
  OffsetPose.msg
  GPSData.msg

  # UWB msg
  LinktrackNode2.msg
  LinktrackNodeframe2.msg

#communication
  DetectionInfoSub.msg
  GimbalControl.msg
  GimbalState.msg
  MultiDetectionInfoSub.msg
  RheaCommunication.msg
  RheaGPS.msg
  RheaState.msg
  VisionDiff.msg
  WindowPosition.msg
	Bspline.msg
	MultiBsplines.msg
)

add_action_files(
  DIRECTORY action
  FILES
  CheckForObjects.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs 
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  std_msgs
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
