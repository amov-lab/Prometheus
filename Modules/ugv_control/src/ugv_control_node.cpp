#include <ros/ros.h>

#include "ugv_controller.h"
#include "ugv_estimator.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[uav_controller_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_controller_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_printf", flag_printf, true);

    UGV_estimator ugv_estimator(nh);
    UGV_controller ugv_controller(nh);

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        ugv_controller.mainloop();

        rate.sleep();
    }

    return 0;
}
