catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/common --build build/common
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
catkin_make --source Modules/uav_control --build build/uav_control
catkin_make --source Modules/matlab_bridge --build build/matlab_bridge
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo
