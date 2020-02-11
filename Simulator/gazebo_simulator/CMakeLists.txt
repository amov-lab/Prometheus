cmake_minimum_required(VERSION 2.8.3)
project(prometheus_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
	rospy
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    cv_bridge
    rostest
    std_msgs
    gazebo_msgs
    prometheus_msgs
    message_generation
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

add_message_files(
	FILES
	IRLock.msg
    Control.msg 
)

generate_messages(DEPENDENCIES 
	std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        gazebo_ros
	std_msgs
	message_runtime
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})


add_library(
    realsense_gazebo_plugin
	gazebo_plugin/RealSensePlugin.cpp
	gazebo_plugin/gazebo_ros_realsense.cpp
)
target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})


add_executable(qrcode_control src/qrcode_control.cpp)
add_dependencies(qrcode_control prometheus_gazebo_gencpp) 
target_link_libraries(qrcode_control ${catkin_LIBRARIES})

#add_library(irlock_gazebo_ros_plugin gazebo_plugin/irlock_gazebo_ros_plugin.cpp)
#target_link_libraries(irlock_gazebo_ros_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(irlock_gazebo_ros_plugin ${catkin_EXPORTED_TARGETS})


#add_library(PriusHybridPlugin SHARED gazebo_plugin/PriusHybridPlugin.cc)
#target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
#add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/joystick_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(PROGRAMS 
   scripts/keyboard_control_px4.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  install(PROGRAMS 
   scripts/qrcode_control.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
