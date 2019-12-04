cmake_minimum_required(VERSION 2.8.3)
project(landpad_det)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)



catkin_package(
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  /usr/include
)

add_executable(landpad_det landpad_det.cpp)
target_link_libraries(landpad_det ${catkin_LIBRARIES}
${OpenCV_LIBS}
${Boost_LIBRARIES}
)
