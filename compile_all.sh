catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/control --build build/control
catkin_make --source Modules/object_detection --build build/object_detection
catkin_make --source Modules/mission --build build/mission
catkin_make --source Modules/slam --build build/slam
catkin_make --source Modules/planning --build build/planning
catkin_make --source Modules/ground_station --build build/ground_station
catkin_make --source Modules/ugv_control --build build/ugv_control
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
catkin_make --source Modules/ego_planner --build build/ego_planner
catkin_make --source Modules/swarm_control --build build/swarm_control
catkin_make --source Experiment --build build/prometheus_experiment


