cmake_minimum_required(VERSION 3.5.0)
add_compile_options(-std=c++11)
project(prometheus_stereomatching)

find_package(catkin REQUIRED
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    tf
)

find_package(OpenCV 3.3.1 REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3/")

find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node 
    nodes/sineva_stereo_node.cpp
    src/descriptor.cpp
    src/elas.cpp
    src/filter.cpp
    src/matrix.cpp
    src/stereomatch.cpp
    src/pointcloud.cpp
    src/triangle.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)


