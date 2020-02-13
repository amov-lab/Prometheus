cmake_minimum_required(VERSION 2.8.3)
project(prometheus_local_planning)

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
       visualization_msgs
	   prometheus_msgs
	)


catkin_package(
       INCLUDE_DIRS include
       LIBRARIES potential_field_planning
       #  DEPENDS system_lib
	)

# SET(USER_KEY, "Hello World")
MESSAGE( STATUS "this var key ******** = ${PROJECT_SOURCE_DIR}.")


include_directories(
       SYSTEM
       include
	   ${PROJECT_SOURCE_DIR}/include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
     )
link_directories(${PCL_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

# add_library(potential_field_planning
#        src/potential_field_planing.cpp
#        )

# target_link_libraries( potential_field_planning
#        ${catkin_LIBRARIES}
# 	   ${PCL_LIBRARIES}
#        )


add_executable(local_planner_main 
	src/planner_node.cpp
	src/potential_field_planing.cpp
	src/apf.cpp
	src/planning_visualization.cpp
	)

target_link_libraries(local_planner_main
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
	)
