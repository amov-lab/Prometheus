#include <ros/ros.h>
#include <signal.h>
#include "swarm_control.h"


void mySigintHandler(int sig)
{
  ROS_INFO("[swarm_control_node] exit...");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_control_node");
  ros::NodeHandle nh_("~");

  //SIGINT表示从键盘中断
  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  SwarmControl swarm_control(nh_);

  // 主循环
  ros::Rate rate(100.0);
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    swarm_control.mainLoop();
  }

  return 0;
}
