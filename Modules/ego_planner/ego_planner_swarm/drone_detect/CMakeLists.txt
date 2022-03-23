cmake_minimum_required(VERSION 2.8.3)
project(drone_detect)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    roslint
    cv_bridge
    message_filters
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

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
  INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
  DEPENDS OpenCV Eigen Boost
  ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
  ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
  # Eigen3

  ## Boost is not part of the DEPENDS since it is only used in source files,
  ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/drone_detector.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# if(${CATKIN_ENABLE_TESTING})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# ## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test
#   test/test_drone_detector.cpp)

# target_link_libraries(${PROJECT_NAME}-test)
# endif()

##########################
## Static code analysis ##
##########################

roslint_cpp()
