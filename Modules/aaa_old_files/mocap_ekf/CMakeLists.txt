cmake_minimum_required(VERSION 2.8.3)
project(mocap_ekf)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

catkin_package(
    )

# add_executable(ekf src/ekf_node.cpp src/conversion.cpp)  # to time synchronization  #oldest one, Euler transform is pnp the world in cam transforms to imu in world
# add_executable(ekf src/ekf_node_mocap.cpp src/conversion.cpp)  # Euler transform is rigidbody in world transforms to imu in world
# add_executable(ekf src/ekf_node_vio.cpp src/conversion.cpp) # only use ekf_node_vio.cpp (ekf_node_mocap.cpp is just for mocap format, base on this one)
# add_executable(ekf src/ekf_node_vio_timesync.cpp src/conversion.cpp)
add_executable(mocap_ekf src/mocap_ekf_node.cpp src/conversion.cpp)
add_executable(imu_add_noise src/imu_add_noise.cpp src/conversion.cpp)
target_link_libraries(mocap_ekf ${catkin_LIBRARIES} )
target_link_libraries(imu_add_noise ${catkin_LIBRARIES} )

