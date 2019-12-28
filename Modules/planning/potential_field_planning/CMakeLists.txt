cmake_minimum_required(VERSION 2.8.3)
project(potential_field_planning)

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
        LIBRARIES potential_field_planning
        CATKIN_DEPENDS plan_env
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
       # /home/taojiang/install/nlopt/include # add include path for nlopt
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( potential_field_planning
        src/potential_field_planing.cpp
        )

target_link_libraries( potential_field_planning
        ${catkin_LIBRARIES}
        #/usr/local/lib/libnlopt.so
        #/home/taojiang/install/nlopt/lib/libnlopt.so
        )
