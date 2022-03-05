cmake_minimum_required(VERSION 2.8.3)
project(prometheus_local_planner)

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
       mavros_msgs
	)


catkin_package(
       INCLUDE_DIRS include
       LIBRARIES LocalPlannerNS
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


add_executable(local_planner_main 
	src/local_planner_node.cpp
	src/local_planner.cpp
       src/apf.cpp
       src/vfh.cpp      
	)

target_link_libraries(local_planner_main
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
	)
