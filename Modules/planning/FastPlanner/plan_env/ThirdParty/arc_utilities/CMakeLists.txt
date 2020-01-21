cmake_minimum_required(VERSION 2.8.3)
project(arc_utilities)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(DIRECTORY msg FILES )

## Generate services in the 'srv' folder
# add_service_files(DIRECTORY srv FILES )

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES geometry_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs DEPENDS Eigen3)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wconversion -Werror")

# Utility library
add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/log.hpp
    include/${PROJECT_NAME}/maybe.hpp
    include/${PROJECT_NAME}/ros_helpers.hpp
    include/${PROJECT_NAME}/abb_irb1600_145_fk_fast.hpp
    include/${PROJECT_NAME}/iiwa_7_fk_fast.hpp
    include/${PROJECT_NAME}/iiwa_14_fk_fast.hpp
    include/${PROJECT_NAME}/arc_exceptions.hpp
    include/${PROJECT_NAME}/arc_helpers.hpp
    include/${PROJECT_NAME}/pretty_print.hpp
    include/${PROJECT_NAME}/zlib_helpers.hpp
    include/${PROJECT_NAME}/base64_helpers.hpp
    include/${PROJECT_NAME}/voxel_grid.hpp
    include/${PROJECT_NAME}/dynamic_spatial_hashed_voxel_grid.hpp
    include/${PROJECT_NAME}/aligned_eigen_types.hpp
    include/${PROJECT_NAME}/eigen_helpers.hpp
    include/${PROJECT_NAME}/eigen_helpers_conversions.hpp
    include/${PROJECT_NAME}/simple_rrt_planner.hpp
    include/${PROJECT_NAME}/simple_astar_planner.hpp
    include/${PROJECT_NAME}/simple_prm_planner.hpp
    include/${PROJECT_NAME}/simple_kmeans_clustering.hpp
    include/${PROJECT_NAME}/simple_hierarchical_clustering.hpp
    include/${PROJECT_NAME}/simple_hausdorff_distance.hpp
    include/${PROJECT_NAME}/simple_dtw.hpp
    include/${PROJECT_NAME}/timing.hpp
    include/${PROJECT_NAME}/dijkstras.hpp
    include/${PROJECT_NAME}/shortcut_smoothing.hpp
    include/${PROJECT_NAME}/first_order_deformation.h
    include/${PROJECT_NAME}/get_neighbours.hpp
    include/${PROJECT_NAME}/serialization.hpp
    include/${PROJECT_NAME}/serialization_eigen.hpp
    include/${PROJECT_NAME}/serialization_ros.hpp
    include/${PROJECT_NAME}/filesystem.hpp
    src/${PROJECT_NAME}/zlib_helpers.cpp
    src/${PROJECT_NAME}/base64_helpers.cpp
    src/timing.cpp
    src/${PROJECT_NAME}/first_order_deformation.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} z)

# Simple test node for simple_hierarchical_clustering
add_executable(test_hierarchical_clustering src/test_hierarchical_clustering.cpp)
add_dependencies(test_hierarchical_clustering ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
target_link_libraries(test_hierarchical_clustering ${PROJECT_NAME} ${catkin_LIBRARIES})

# Simple test node for utility functions
add_executable(test_arc_utilities src/test_arc_utilities.cpp)
add_dependencies(test_arc_utilities ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
target_link_libraries(test_arc_utilities ${PROJECT_NAME} ${catkin_LIBRARIES})

# # Simple test node for Eigen3 math
# add_executable(test_eigen_math src/test_eigen_math.cpp)
# add_dependencies(test_eigen_math ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
# target_link_libraries(test_eigen_math ${PROJECT_NAME} ${catkin_LIBRARIES})

# # Simple test node for averaging math
# add_executable(test_averaging src/test_averaging.cpp)
# add_dependencies(test_averaging ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
# target_link_libraries(test_averaging ${PROJECT_NAME} ${catkin_LIBRARIES})

# # Simple test node for Dijkstras
# add_executable(test_dijkstras src/test_dijkstras.cpp)
# add_dependencies(test_dijkstras ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
# target_link_libraries(test_dijkstras ${PROJECT_NAME} ${catkin_LIBRARIES})

# # Simple test node for shortcut smoothing
# add_executable(test_shortcut_smoothing src/test_shortcut_smoothing.cpp)
# add_dependencies(test_shortcut_smoothing ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
# target_link_libraries(test_shortcut_smoothing ${PROJECT_NAME} ${catkin_LIBRARIES})

# # Simple test node for DistanceToLine function
# add_executable(test_closest_point src/test_closest_point.cpp)
# add_dependencies(test_closest_point ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # ${PROJECT_NAME}_gencpp)
# target_link_libraries(test_closest_point ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark library for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
