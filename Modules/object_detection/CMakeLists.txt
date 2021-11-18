cmake_minimum_required(VERSION 2.8.3)
project(prometheus_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# set(OpenCV_DIR /usr/local/share/OpenCV)

# Define path of darknet folder here.
find_path(DARKNET_PATH
  NAMES "Makefile"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_lib/")
message("Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

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

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src/kcf_tracker_lib
  src/ellipse_lib
  src/darknet_lib/src
  src/darknet_lib/include
  src/darknet_ros/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ../common/include
)


if (CUDA_FOUND)

  link_directories(
    /usr/local/cuda/lib64
  )

  cuda_add_library(darknet_ros_lib
    src/darknet_ros/src/YoloObjectDetector.cpp
    src/darknet_ros/src/image_interface.c

    src/darknet_lib/src/activation_layer.c        src/darknet_lib/src/im2col.c
    src/darknet_lib/src/activations.c             src/darknet_lib/src/image.c
    src/darknet_lib/src/avgpool_layer.c           src/darknet_lib/src/layer.c
    src/darknet_lib/src/batchnorm_layer.c         src/darknet_lib/src/list.c
    src/darknet_lib/src/blas.c                    src/darknet_lib/src/local_layer.c
    src/darknet_lib/src/box.c                     src/darknet_lib/src/lstm_layer.c
    src/darknet_lib/src/col2im.c                  src/darknet_lib/src/matrix.c
    src/darknet_lib/src/connected_layer.c         src/darknet_lib/src/maxpool_layer.c
    src/darknet_lib/src/convolutional_layer.c     src/darknet_lib/src/network.c
    src/darknet_lib/src/cost_layer.c              src/darknet_lib/src/normalization_layer.c
    src/darknet_lib/src/crnn_layer.c              src/darknet_lib/src/option_list.c
    src/darknet_lib/src/crop_layer.c              src/darknet_lib/src/parser.c
    src/darknet_lib/src/cuda.c                    src/darknet_lib/src/region_layer.c
    src/darknet_lib/src/data.c                    src/darknet_lib/src/reorg_layer.c
    src/darknet_lib/src/deconvolutional_layer.c   src/darknet_lib/src/rnn_layer.c
    src/darknet_lib/src/demo.c                    src/darknet_lib/src/route_layer.c
    src/darknet_lib/src/detection_layer.c         src/darknet_lib/src/shortcut_layer.c
    src/darknet_lib/src/dropout_layer.c           src/darknet_lib/src/softmax_layer.c
    src/darknet_lib/src/gemm.c                    src/darknet_lib/src/tree.c
    src/darknet_lib/src/gru_layer.c               src/darknet_lib/src/utils.c
    src/darknet_lib/src/upsample_layer.c          src/darknet_lib/src/logistic_layer.c
    src/darknet_lib/src/l2norm_layer.c            src/darknet_lib/src/yolo_layer.c

    src/darknet_lib/examples/art.c                src/darknet_lib/examples/lsd.c
    src/darknet_lib/examples/attention.c          src/darknet_lib/examples/nightmare.c
    src/darknet_lib/examples/captcha.c            src/darknet_lib/examples/regressor.c
    src/darknet_lib/examples/cifar.c              src/darknet_lib/examples/rnn.c
    src/darknet_lib/examples/classifier.c         src/darknet_lib/examples/segmenter.c
    src/darknet_lib/examples/coco.c               src/darknet_lib/examples/super.c
    src/darknet_lib/examples/darknet.c            src/darknet_lib/examples/tag.c
    src/darknet_lib/examples/detector.c           src/darknet_lib/examples/yolo.c
    src/darknet_lib/examples/go.c

    src/darknet_lib/src/activation_kernels.cu     src/darknet_lib/src/crop_layer_kernels.cu
    src/darknet_lib/src/avgpool_layer_kernels.cu  src/darknet_lib/src/deconvolutional_kernels.cu
    src/darknet_lib/src/blas_kernels.cu           src/darknet_lib/src/dropout_layer_kernels.cu
    src/darknet_lib/src/col2im_kernels.cu         src/darknet_lib/src/im2col_kernels.cu
    src/darknet_lib/src/convolutional_kernels.cu  src/darknet_lib/src/maxpool_layer_kernels.cu
  )

  target_link_libraries(darknet_ros_lib
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  cuda_add_executable(darknet_ros
    src/darknet_ros/src/yolo_object_detector_node.cpp
  )

else()

  add_library(darknet_ros_lib
    src/darknet_ros/src/YoloObjectDetector.cpp
    src/darknet_ros/src/image_interface.c

    src/darknet_lib/src/activation_layer.c        src/darknet_lib/src/im2col.c
    src/darknet_lib/src/activations.c             src/darknet_lib/src/image.c
    src/darknet_lib/src/avgpool_layer.c           src/darknet_lib/src/layer.c
    src/darknet_lib/src/batchnorm_layer.c         src/darknet_lib/src/list.c
    src/darknet_lib/src/blas.c                    src/darknet_lib/src/local_layer.c
    src/darknet_lib/src/box.c                     src/darknet_lib/src/lstm_layer.c
    src/darknet_lib/src/col2im.c                  src/darknet_lib/src/matrix.c
    src/darknet_lib/src/connected_layer.c         src/darknet_lib/src/maxpool_layer.c
    src/darknet_lib/src/convolutional_layer.c     src/darknet_lib/src/network.c
    src/darknet_lib/src/cost_layer.c              src/darknet_lib/src/normalization_layer.c
    src/darknet_lib/src/crnn_layer.c              src/darknet_lib/src/option_list.c
    src/darknet_lib/src/crop_layer.c              src/darknet_lib/src/parser.c
    src/darknet_lib/src/cuda.c                    src/darknet_lib/src/region_layer.c
    src/darknet_lib/src/data.c                    src/darknet_lib/src/reorg_layer.c
    src/darknet_lib/src/deconvolutional_layer.c   src/darknet_lib/src/rnn_layer.c
    src/darknet_lib/src/demo.c                    src/darknet_lib/src/route_layer.c
    src/darknet_lib/src/detection_layer.c         src/darknet_lib/src/shortcut_layer.c
    src/darknet_lib/src/dropout_layer.c           src/darknet_lib/src/softmax_layer.c
    src/darknet_lib/src/gemm.c                    src/darknet_lib/src/tree.c
    src/darknet_lib/src/gru_layer.c               src/darknet_lib/src/utils.c
    src/darknet_lib/src/upsample_layer.c          src/darknet_lib/src/logistic_layer.c
    src/darknet_lib/src/l2norm_layer.c            src/darknet_lib/src/yolo_layer.c

    src/darknet_lib/examples/art.c                src/darknet_lib/examples/lsd.c
    src/darknet_lib/examples/attention.c          src/darknet_lib/examples/nightmare.c
    src/darknet_lib/examples/captcha.c            src/darknet_lib/examples/regressor.c
    src/darknet_lib/examples/cifar.c              src/darknet_lib/examples/rnn.c
    src/darknet_lib/examples/classifier.c         src/darknet_lib/examples/segmenter.c
    src/darknet_lib/examples/coco.c               src/darknet_lib/examples/super.c
    src/darknet_lib/examples/darknet.c            src/darknet_lib/examples/tag.c
    src/darknet_lib/examples/detector.c           src/darknet_lib/examples/yolo.c
    src/darknet_lib/examples/go.c
  )

  target_link_libraries(darknet_ros_lib
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  add_executable(darknet_ros
    src/darknet_ros/src/yolo_object_detector_node.cpp
  )

endif()

target_link_libraries(darknet_ros
  darknet_ros_lib
)

add_dependencies(darknet_ros_lib
  darknet_ros_msgs_generate_messages_cpp
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

add_library(ellipselib
    src/ellipse_lib/spire_ellipsedetector.cpp
    src/ellipse_lib/spire_ellipsedetector.h
)
target_link_libraries(ellipselib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

## Nodes
add_executable(web_cam
    cpp_nodes/web_cam.cpp
)
target_link_libraries(web_cam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(kcf_tracker
    cpp_nodes/kcf_tracker.cpp
)
target_link_libraries(kcf_tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} kcftrackerlib yaml-cpp)

add_executable(ellipse_det
    cpp_nodes/ellipse_det.cpp
)
target_link_libraries(ellipse_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ellipselib yaml-cpp)

add_executable(aruco_det
    cpp_nodes/aruco_det.cpp
)
target_link_libraries(aruco_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(aruco_det_v2
    cpp_nodes/aruco_det_v2.cpp
)
target_link_libraries(aruco_det_v2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp Eigen3::Eigen)

add_executable(create_board_charuco
    cpp_nodes/create_board_charuco.cpp
)
target_link_libraries(create_board_charuco ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(calibrate_camera_charuco
    cpp_nodes/calibrate_camera_charuco.cpp
)
target_link_libraries(calibrate_camera_charuco ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(landpad_det
    cpp_nodes/landpad_det.cpp
)
target_link_libraries(landpad_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(stereo_detection_saving_gazebo_samples
    cpp_nodes/3d_detection_saving_gazebo_samples.cpp
)
target_link_libraries(stereo_detection_saving_gazebo_samples ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(aruco_navigation_switch
    cpp_nodes/aruco_navigation_switch.cpp
)
target_link_libraries(aruco_navigation_switch ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

if(CATKIN_ENABLE_TESTING)
  # Download yolov3-tiny.weights
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_ros/yolo_network_config/weights")
  set(FILE "${PATH}/yolov3-tiny.weights")
  message(STATUS "Checking and downloading yolov3-tiny.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget http://jario.ren/upload/yolov3-tiny.weights -P ${PATH})
  endif()
  # Download yolov3-tiny-BDT-best.weights
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_ros/yolo_network_config/weights")
  set(FILE "${PATH}/yolov3-tiny-BDT-best.weights")
  message(STATUS "Checking and downloading yolov3-tiny-BDT-best.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget http://jario.ren/upload/yolov3-tiny-BDT-best.weights -P ${PATH})
  endif()
  # Download SiamRPNVOT.model
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/siam_rpn_lib")
  set(FILE "${PATH}/SiamRPNVOT.model")
  message(STATUS "Checking and downloading SiamRPNVOT.model if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget http://jario.ren/upload/SiamRPNVOT.model -P ${PATH})
  endif()
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fly_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
