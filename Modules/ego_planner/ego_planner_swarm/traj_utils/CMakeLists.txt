cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  #bspline_opt
  #path_searching
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  #cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  MultiBsplines.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils 
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp 
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  
