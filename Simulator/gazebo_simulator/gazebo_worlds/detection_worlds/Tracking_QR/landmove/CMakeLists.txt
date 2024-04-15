cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
 
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
 
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
 
 
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
 
# 使用line.cc源文件生成插件，生成libline.so库文件
add_library(landmove SHARED landmove.cc)          
 
# 在生成插件过程中需要使用到的gazebo库文件
target_link_libraries(landmove ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
