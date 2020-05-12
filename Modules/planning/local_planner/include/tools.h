#ifndef _TOOLS_H
#define _TOOLS_H

#include <iostream>
#include <algorithm>

// #include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "prometheus_msgs/Message.h"

namespace local_planner
{
// void pub_msg(ros::Publisher & puber, std::string mmm, int type);

extern ros::Publisher message_pub;
}
#endif