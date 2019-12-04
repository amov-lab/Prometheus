cmake_minimum_required(VERSION 2.8.3)
project(web_cam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge

)

find_package(OpenCV REQUIRED)

catkin_package(
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  /usr/include
)

add_executable(web_cam web_cam.cpp)
target_link_libraries(web_cam ${catkin_LIBRARIES}
${OpenCV_LIBS}
)
