#include <ros/ros.h>
#include <signal.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);

    ros::WallTime wall_time_now;
    ros::Time time_now;
    int index = 1;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        // 定时状态打印
        wall_time_now = ros::WallTime::now();
        time_now = ros::Time::now();

        ROS_INFO("index: %d", index);
        ROS_INFO("time_now: %f", time_now.toSec());
        ROS_INFO("wall_time_now: %f", wall_time_now.toSec());


        index++;
        rate.sleep();
    }

    return 0;
}
