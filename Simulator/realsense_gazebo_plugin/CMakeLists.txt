cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  roscpp
  nodelet
  sensor_msgs
  urdf
  tf
  tf2_ros
  rosconsole
  diagnostic_updater
  std_msgs
  gazebo_ros
  image_transport
  camera_info_manager
)

find_package(Boost REQUIRED COMPONENTS thread)
# find_package(gazebo REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  nodelet
  geometry_msgs
  sensor_msgs
  urdf
  tf
  tf2_ros
  rosconsole
  std_msgs
  gazebo_ros
  image_transport
  camera_info_manager
)

###########
## Build ##
###########

add_library(realsense_gazebo_plugin src/RealSensePlugin.cpp src/gazebo_ros_realsense.cpp)
target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  realsense_gazebo_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY xacro
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
