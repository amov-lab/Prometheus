# Top-level CMakeLists.txt for Prometheus catkin workspace
# This file enables `catkin_make` from the project root.
#
# Usage:
#   catkin_make                          # Build all modules
#   catkin_make --only-pkg-with-deps prometheus_uav_control  # Build specific package
#
# For module-specific builds, use the scripts in scripts/build/

cmake_minimum_required(VERSION 2.8.12)
find_package(catkin REQUIRED)
catkin_workspace()
