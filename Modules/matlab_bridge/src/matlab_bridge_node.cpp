#include <ros/ros.h>

#include "matlab_bridge.h"
#include <signal.h>

void mySigintHandler(int sig)
{
  ROS_INFO("[matlab_bridge_node] exit...");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "matlab_bridge_node");
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  Matlab_Bridge matlab_bridge(nh);
  
  ros::Duration(1.0).sleep();

  // 主循环
  ros::Rate rate(100.0);
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    // 主循环函数
    matlab_bridge.mainloop();
  }

  return 0;
}

