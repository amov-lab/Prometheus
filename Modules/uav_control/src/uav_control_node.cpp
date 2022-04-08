#include <ros/ros.h>

#include "uav_controller.h"
#include "uav_estimator.h"
#include <signal.h>

void mySigintHandler(int sig)
{
  ROS_INFO("[uav_controller_node] exit...");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_controller_node");
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  bool sim_mode;
  nh.param<bool>("sim_mode", sim_mode, true);

  UAV_estimator uav_estimator(nh);
  UAV_controller uav_controller(nh);
  
  ros::Duration(1.0).sleep();

  // 检查PX4连接状态
  while (ros::ok() && !uav_estimator.uav_state.connected)
  {
    ros::spinOnce();
    ros::Duration(5.0).sleep();
    ROS_ERROR("Waiting for connect PX4!");
  }

  ros::Duration(2.0).sleep();

  // 主循环
  ros::Rate rate(100.0);
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    // 主循环函数
    uav_controller.mainloop();
  }

  return 0;
}

