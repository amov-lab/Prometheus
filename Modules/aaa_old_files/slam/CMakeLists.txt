cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(prometheus_slam)

find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED
    message_generation
    message_runtime
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    tf
)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3/")


find_package( PCL REQUIRED COMPONENT common io )
add_definitions( ${PCL_DEFINITIONS} )
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node 
    src/stereo_matching/elas_stereo_node.cpp
    src/stereo_matching/lib/descriptor.cpp
    src/stereo_matching/lib/elas.cpp
    src/stereo_matching/lib/filter.cpp
    src/stereo_matching/lib/matrix.cpp
    src/stereo_matching/lib/stereomatch.cpp
    src/stereo_matching/lib/pointcloud.cpp
    src/stereo_matching/lib/triangle.cpp
)
add_executable(map_building_node src/map_building/pointcloud_to_octomap_node.cpp)

add_executable(pc2_publisher_node src/map_building/pointcloud2_publisher.cpp)

add_executable(pc2_saver_node src/map_building/pointcloud2_saver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES} 
)
target_link_libraries(map_building_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES} 
)
target_link_libraries(pc2_publisher_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(pc2_saver_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)


