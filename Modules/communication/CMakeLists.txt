cmake_minimum_required(VERSION 3.0.2)
project(prometheus_communication_bridge)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

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
find_package(PCL 1.7 REQUIRED)

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
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/shard/include
  ${PROJECT_SOURCE_DIR}/shard/include/json/include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

file(GLOB_RECURSE CURRENT_INCLUDE include/*.hpp include/*.h)
file(GLOB_RECURSE CURRENT_SOURCE src/*.cpp)

# 获取Ubuntu版本号
execute_process(
    COMMAND lsb_release -rs
    OUTPUT_VARIABLE UBUNTU_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

#add_definitions(${PCL_DEFINITIONS})
## Specify libraries to link a library or executable target against
add_executable(communication_bridge ${CURRENT_SOURCE} ${CURRENT_INCLUDE})
add_dependencies(communication_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_libraries(communication_bridge ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/shard/libs/libcommunication_x86_64.so ${PCL_LIBRARIES})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(communication_bridge ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/shard/libs/libcommunication_aarch64.so ${PCL_LIBRARIES})
endif()




