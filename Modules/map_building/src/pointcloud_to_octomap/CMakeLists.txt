cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(prometheus_octomap_builder)

find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(LIBS 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}  
)

add_executable(PointCloudToOctomapNode
  src/pointcloud_to_octomap_node.cpp
)

target_link_libraries(PointCloudToOctomapNode
  ${LIBS}
)

install(TARGETS PointCloudToOctomapNode
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
