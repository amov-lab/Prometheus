# Prometheus Build System
# Usage:
#   make all         - Build all modules
#   make core        - Build core (common messages/utils)
#   make control     - Build control modules
#   make planning    - Build planning modules
#   make perception  - Build perception modules
#   make communication - Build communication module
#   make swarm       - Build swarm modules
#   make demo        - Build demo modules
#   make simulator   - Build simulator modules
#   make clean       - Clean all build artifacts

SHELL := /bin/bash

.PHONY: all core control planning perception communication swarm demo simulator clean help

help:
	@echo "Prometheus Build Targets:"
	@echo "  make all            - Build all modules"
	@echo "  make core           - Build core (messages & utils)"
	@echo "  make control        - Build UAV/UGV control"
	@echo "  make planning       - Build path planning modules"
	@echo "  make perception     - Build perception modules (FAST_LIO, etc.)"
	@echo "  make communication  - Build communication bridge"
	@echo "  make swarm          - Build swarm modules"
	@echo "  make demo           - Build tutorial & demo modules"
	@echo "  make simulator      - Build Gazebo simulator"
	@echo "  make clean          - Remove build/ and devel/"
	@echo ""
	@echo "Individual build scripts are in scripts/build/"

# Core messages - must be built first
core:
	catkin_make --source Modules/core/common --build build/common

# Simulator modules
simulator: core
	catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
	catkin_make --source Simulator/realsense_gazebo_plugin --build build/realsense_gazebo_plugin
	catkin_make --source Simulator/velodyne_gazebo_plugins --build build/velodyne_gazebo_plugins
	catkin_make --source Simulator/livox_laser_gazebo_plugins --build build/livox_laser_gazebo_plugins

# Control modules
control: core simulator
	catkin_make --source Modules/control/uav_control --build build/uav_control

# Communication module
communication: core
	catkin_make --source Modules/communication --build build/communication

# Planning modules
planning: core control
	catkin_make --source Modules/perception/simulator_utils --build build/simulator_utils
	catkin_make --source Modules/planning/ego_planner_swarm --build build/ego_planner_swarm
	catkin_make --source Modules/planning/motion_planning --build build/motion_planning

# Perception modules
perception: core
	catkin_make --source Modules/perception/FAST_LIO --build build/FAST_LIO

# Demo modules
demo: core control
	catkin_make --source Modules/demo/tutorial_demo --build build/tutorial_demo

# Swarm modules
swarm: core communication control
	catkin_make --source Modules/control/ugv_control --build build/ugv_control
	catkin_make --source Modules/swarm/swarm_control --build build/swarm_control
	catkin_make --source Modules/swarm/swarm_formation --build build/swarm_formation
	catkin_make --source Modules/swarm/searching_pkg --build build/searching_pkg

# Build everything
all: core communication simulator control demo planning perception

clean:
	rm -rf build/ devel/
	@echo "Build artifacts cleaned."
