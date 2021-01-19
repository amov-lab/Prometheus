catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Modules/control --build build/control
catkin_make --source Modules/object_detection --build build/object_detection
catkin_make --source Modules/mission --build build/mission
catkin_make --source Modules/ground_station --build build/ground_station
catkin_make --source Experiment --build build/prometheus_experiment

