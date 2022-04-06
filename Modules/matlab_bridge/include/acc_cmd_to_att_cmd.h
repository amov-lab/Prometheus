//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "printf_utils.h"

using namespace std;

#define MATLAB_TIMEOUT 0.2
#define RETURN_INIT_POS_TIMEOUT 60
#define LAND_TIMEOUT 300



