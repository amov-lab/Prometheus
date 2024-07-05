cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  std_msgs
  message_generation
  prometheus_msgs
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
include_directories(
         include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_gazebo_plugins
        CATKIN_DEPENDS tf
)

add_library(livox_laser_gazebo_plugins SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_gazebo_plugins ${catkin_LIBRARIES} RayPlugin)

target_link_libraries(livox_laser_gazebo_plugins libprotobuf.so.9)
