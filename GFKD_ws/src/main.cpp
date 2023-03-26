#include <ros/ros.h>
#include <signal.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);

    ros::Time time_now;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        // 定时状态打印
        time_now = ros::Time::now();


        ROS_INFO("time_now: %f",time_now.toSec());


        rate.sleep();
    }

    return 0;
}
