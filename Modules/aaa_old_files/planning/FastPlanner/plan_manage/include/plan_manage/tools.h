#ifndef _TOOLS_H
#define _TOOLS_H

#include <iostream>
#include <algorithm>

// #include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "prometheus_msgs/Message.h"

namespace dyn_planner
{
extern ros::Publisher message_pub;

// void pub_msg(ros::Publisher & puber, std::string mmm, int type);
}
#endif