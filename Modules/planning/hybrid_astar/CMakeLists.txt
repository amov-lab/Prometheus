cmake_minimum_required(VERSION 2.8.3)
project(prometheus_hybrid_astar)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
	   sensor_msgs
	   geometry_msgs
	   nav_msgs
	   pcl_ros
       visualization_msgs
	   prometheus_msgs
	)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES Global_Planning
	#  DEPENDS system_lib
)

include_directories(
       SYSTEM
       include
	   ${PROJECT_SOURCE_DIR}/include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
	   ${PROJECT_SOURCE_DIR}/../../common/include
     )
link_directories(${PCL_LIBRARY_DIRS})


add_executable(hybrid_astar
	src/global_planner_node.cpp
	src/global_planner.cpp
	src/kinodynamic_astar.cpp
	src/occupy_map.cpp
	src/tools.cpp
	)

target_link_libraries(hybrid_astar
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
	)