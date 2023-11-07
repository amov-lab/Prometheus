cmake_minimum_required(VERSION 2.8.3)
project(prometheus_case2_ugv)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        laser_geometry
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_ros
        visualization_msgs
        prometheus_msgs
        tf2
)


catkin_package(
	INCLUDE_DIRS include
	#  DEPENDS system_lib
)

include_directories(
       SYSTEM
       include
	   ${PROJECT_SOURCE_DIR}/include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
	   ${PROJECT_SOURCE_DIR}/../common/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(occupy_map src/occupy_map.cpp)
target_link_libraries(occupy_map ${PCL_LIBRARIES})
add_library(A_star src/A_star.cpp)
target_link_libraries(A_star occupy_map)
add_library(case2_fsm_ugv src/case2_fsm_ugv.cpp)
target_link_libraries(case2_fsm_ugv A_star)
target_link_libraries(case2_fsm_ugv ${catkin_LIBRARIES})
target_link_libraries(case2_fsm_ugv ${PCL_LIBRARIES})

add_executable(case2_ugv_main src/case2_ugv_node.cpp)
add_dependencies(case2_ugv_main prometheus_case2_ugv_gencpp)
target_link_libraries(case2_ugv_main ${catkin_LIBRARIES})
target_link_libraries(case2_ugv_main ${PCL_LIBRARIES})
target_link_libraries(case2_ugv_main case2_fsm_ugv)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
