cmake_minimum_required(VERSION 2.8.3)
project(sdf_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs visualization_msgs image_transport cv_bridge arc_utilities message_generation)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)
# MoveIt! isn't a required dependency
find_package(moveit_msgs)
find_package(moveit_core)
find_package(moveit_ros_planning)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES SDF.msg CollisionMap.msg TaggedObjectCollisionMap.msg)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES ComputeSDF.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if(moveit_core_FOUND)
    catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS rospy std_msgs sensor_msgs moveit_core moveit_ros_planning visualization_msgs moveit_msgs image_transport cv_bridge arc_utilities message_runtime DEPENDS Eigen3 OpenCV)
else()
    catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS rospy std_msgs sensor_msgs visualization_msgs image_transport cv_bridge arc_utilities message_runtime DEPENDS Eigen3 OpenCV)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if(moveit_core_FOUND)
    include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${moveit_core_INCLUDE_DIRS} ${moveit_msgs_INCLUDE_DIRS} ${moveit_ros_planning_INCLUDE_DIRS})
else()
    include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -flto -O3 -Wall -Wextra -Werror")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if(moveit_core_FOUND)
    message(STATUS "MoveIt! found. Building SDF Tools library with MoveIt! integration.")
    # SDF library
    add_library(${PROJECT_NAME}
        include/${PROJECT_NAME}/collision_map.hpp
        include/${PROJECT_NAME}/tagged_object_collision_map.hpp
        include/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.hpp
        include/${PROJECT_NAME}/sdf.hpp
        include/${PROJECT_NAME}/sdf_builder.hpp
        src/${PROJECT_NAME}/collision_map.cpp
        src/${PROJECT_NAME}/tagged_object_collision_map.cpp
        src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
        src/${PROJECT_NAME}/sdf.cpp
        src/${PROJECT_NAME}/sdf_builder.cpp)
    add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${moveit_core_LIBRARIES} ${moveit_msgs_LIBRARIES} ${moveit_ros_planning_LIBRARIES})
    # SDF generation node
    add_executable(sdf_generation_node src/sdf_generation_node.cpp)
    add_dependencies(sdf_generation_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
    target_link_libraries(sdf_generation_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${moveit_core_LIBRARIES} ${moveit_msgs_LIBRARIES} ${moveit_ros_planning_LIBRARIES})
else()
    message(STATUS "MoveIt! not found. Building SDF Tools library without MoveIt! integration.")
    # SDF library
    add_library(${PROJECT_NAME}
        include/${PROJECT_NAME}/collision_map.hpp
        include/${PROJECT_NAME}/tagged_object_collision_map.hpp
        include/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.hpp
        include/${PROJECT_NAME}/sdf.hpp
        src/${PROJECT_NAME}/collision_map.cpp
        src/${PROJECT_NAME}/tagged_object_collision_map.cpp
        src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
        src/${PROJECT_NAME}/sdf.cpp)
    add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

# Simple test node for voxel_grid
# add_executable(voxel_grid_test src/test_voxel_grid.cpp)
# add_dependencies(voxel_grid_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
# target_link_libraries(voxel_grid_test ${catkin_LIBRARIES} ${PROJECT_NAME})

# Image to SDF node
# add_executable(image_2d_sdf_node src/image_2d_sdf_node.cpp)
# add_dependencies(image_2d_sdf_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(image_2d_sdf_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Tutorial on how to use the components of the sdf_tools library
# add_executable(sdf_tools_tutorial src/sdf_tools_tutorial.cpp)
# add_dependencies(sdf_tools_tutorial ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
# target_link_libraries(sdf_tools_tutorial ${catkin_LIBRARIES} ${PROJECT_NAME})

#############
## Install ##
#############

## Mark library for installation
install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/lib
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/lib
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${PROJECT_SOURCE_DIR}/include/sdf_tools
  PATTERN ".svn" EXCLUDE
)
