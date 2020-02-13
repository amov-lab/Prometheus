cmake_minimum_required(VERSION 2.8.3)
project(prometheus_uav_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf
  prometheus_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_sim src/uav_sim.cpp)

target_link_libraries(uav_sim 
   ${catkin_LIBRARIES}
)

