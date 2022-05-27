cmake_minimum_required(VERSION 2.8.3)
project(prometheus_global_planner)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	laser_geometry
	geometry_msgs
	nav_msgs
	pcl_ros
	visualization_msgs
	prometheus_msgs
)


catkin_package(
	INCLUDE_DIRS include
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

add_library(occupy_map src/occupy_map.cpp)
target_link_libraries(occupy_map ${PCL_LIBRARIES})
add_library(A_star src/A_star.cpp)
target_link_libraries(A_star occupy_map)
add_library(global_planner src/global_planner.cpp)
target_link_libraries(global_planner A_star)

add_executable(global_planner_main src/global_planner_node.cpp)
add_dependencies(global_planner_main prometheus_global_planner_gencpp)
target_link_libraries(global_planner_main ${catkin_LIBRARIES})
target_link_libraries(global_planner_main ${PCL_LIBRARIES})
target_link_libraries(global_planner_main global_planner)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)