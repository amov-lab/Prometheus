cmake_minimum_required(VERSION 2.8.3)
project(prometheus_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


# Find CUDA
FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find X11
MESSAGE ( STATUS "Searching for X11..." )
FIND_PACKAGE ( X11 REQUIRED )
IF ( X11_FOUND )
  INCLUDE_DIRECTORIES ( ${X11_INCLUDE_DIR} )
  LINK_LIBRARIES ( ${X11_LIBRARIES} )
  MESSAGE ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  MESSAGE ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
ENDIF ( X11_FOUND )


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    rostime
    roscpp
    rosconsole
    roscpp_serialization
    sensor_msgs
    image_transport
    cv_bridge
    actionlib
    prometheus_msgs
    tf2_eigen
    tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(
  src/kcf_tracker_lib
  src/ellipse_lib
  ${catkin_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ../common/include
)


## Declare a C++ library

## Libs
add_library(kcftrackerlib
    src/kcf_tracker_lib/kcftracker.hpp
    src/kcf_tracker_lib/ffttools.hpp
    src/kcf_tracker_lib/fhog.hpp
    src/kcf_tracker_lib/labdata.hpp
    src/kcf_tracker_lib/recttools.hpp
    src/kcf_tracker_lib/tracker.h
    src/kcf_tracker_lib/kcftracker.cpp
    src/kcf_tracker_lib/fhog.cpp
)
target_link_libraries(kcftrackerlib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

## Nodes
add_executable(web_cam
    cpp_nodes/web_cam.cpp
)
target_link_libraries(web_cam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(kcf_tracker
    cpp_nodes/kcf_tracker.cpp
)
target_link_libraries(kcf_tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} kcftrackerlib yaml-cpp)

add_executable(aruco_det
    cpp_nodes/aruco_det.cpp
)
target_link_libraries(aruco_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp Eigen3::Eigen)

add_executable(create_board_charuco
    cpp_nodes/create_board_charuco.cpp
)
target_link_libraries(create_board_charuco ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(calibrate_camera_charuco
    cpp_nodes/calibrate_camera_charuco.cpp
)
target_link_libraries(calibrate_camera_charuco ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(aruco_navigation_switch
    cpp_nodes/aruco_navigation_switch.cpp
)
target_link_libraries(aruco_navigation_switch ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(markers_landpad_det
    cpp_nodes/markers_landpad_det.cpp
)
target_link_libraries(markers_landpad_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(ellipse_det
    cpp_nodes/ellipse_det.cpp
    src/ellipse_detector/ellipse_detector.cpp
    src/ellipse_detector/ellipse_detector.h
)
target_include_directories(ellipse_det PUBLIC
    src/ellipse_detector/
)
target_link_libraries(ellipse_det ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


if(CATKIN_ENABLE_TESTING)
  # Download SiamRPNVOT.model
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/siam_rpn_lib")
  set(FILE "${PATH}/SiamRPNVOT.model")
  message(STATUS "Checking and downloading SiamRPNVOT.model if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget http://jario.ren/upload/SiamRPNVOT.model -P ${PATH})
  endif()
endif()
