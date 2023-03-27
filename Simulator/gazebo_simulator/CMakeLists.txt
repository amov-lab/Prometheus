cmake_minimum_required(VERSION 2.8.3)
project(prometheus_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  message_generation      
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  gazebo_msgs
  prometheus_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
###############################
##      声明可执行cpp文件     ##
###############################
add_executable(set_model_state_example cpp_nodes/set_model_state_example.cpp)
add_dependencies(set_model_state_example prometheus_gazebo_gencpp)
target_link_libraries(set_model_state_example ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)