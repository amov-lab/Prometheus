cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../ThirdParty/nlopt/include # add include path for nlopt
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")


message(STATUS "Project: SERVER")
message(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")

add_library( bspline_opt
    src/bspline_optimizer.cpp 
    src/non_uniform_bspline.cpp
        ../potential_field_planning/src/potential_field_planing.cpp ../potential_field_planning/include/potential_field_planning.h)
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../ThirdParty/nlopt/lib/libnlopt.so
    )  
