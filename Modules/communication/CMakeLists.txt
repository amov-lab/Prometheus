cmake_minimum_required(VERSION 3.0.2)
project(prometheus_communication_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  message_generation
  tf2_msgs
  visualization_msgs
  mavros
  mavros_msgs
  prometheus_msgs
)


generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2_msgs
  visualization_msgs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  ##CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  image2rtsp/include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/shard/include

  ${x264_INCLUDE_DIR}

  ${CMAKE_CURRENT_SOURCE_DIR}image2rtsp/include/live555/BasicUsageEnvironment
  ${CMAKE_CURRENT_SOURCE_DIR}image2rtsp/include/live555/groupsock
  ${CMAKE_CURRENT_SOURCE_DIR}image2rtsp/include/live555/liveMedia
  ${CMAKE_CURRENT_SOURCE_DIR}image2rtsp/include/live555/UsageEnvironment
)

file(GLOB_RECURSE CURRENT_INCLUDE include/*.hpp include/*.h)
file(GLOB_RECURSE CURRENT_SOURCE src/*.cpp)

## Specify libraries to link a library or executable target against
add_executable(communication_bridge ${CURRENT_SOURCE} ${CURRENT_INCLUDE})
add_dependencies(communication_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
target_link_libraries(communication_bridge ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/shard/libs/libcommunication_x86_64.so boost_serialization)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
target_link_libraries(communication_bridge ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/shard/libs/libcommunication_arm.so boost_serialization)
endif()


EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if( ${ARCHITECTURE} STREQUAL "aarch64" )
    set( LIVE555_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/image2rtsp/lib/arm_64" )
else()
    set( LIVE555_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/image2rtsp/lib/x86_64" )
endif()

link_directories(
  ${x264_LIBRARY_DIR}
  ${LIVE555_LIBRARY_DIR}
)

add_executable(image2rtsp_node image2rtsp/src/node.cpp)

set_target_properties(image2rtsp_node PROPERTIES OUTPUT_NAME image2rtsp_node PREFIX "")

target_link_libraries(image2rtsp_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  
  x264
  swscale
  avformat
  avcodec
  avutil

  groupsock
  liveMedia
  BasicUsageEnvironment
  UsageEnvironment
)