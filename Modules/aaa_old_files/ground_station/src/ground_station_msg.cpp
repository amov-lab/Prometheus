//头文件
#include <ros/ros.h>

//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "message_utils.h"

using namespace std;
//---------------------------------------相关参数-----------------------------------------------

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void msg_main_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    printf_message(message);
}
void msg_planning_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    printf_message(message);
}
void msg_det_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    printf_message(message);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station_msg");
    ros::NodeHandle nh("~");

    // 【订阅】 各类消息
    ros::Subscriber message_main_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/main", 10, msg_planning_cb);
    ros::Subscriber message_local_planner_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/local_planner", 10, msg_planning_cb);
    ros::Subscriber message_global_planner_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/global_planner", 10, msg_planning_cb);
    ros::Subscriber message_fast_planner_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/fast_planner", 10, msg_planning_cb);

    ros::Subscriber num_det_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/num_det", 10, msg_det_cb);
    ros::Subscriber circle_det_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/circle_det", 10, msg_det_cb);
    ros::Subscriber pad_det_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/landpad_det", 10, msg_det_cb);
    ros::Subscriber kcf_det_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/kcf_det", 10, msg_det_cb);

    // 频率
    ros::Rate rate(1.0);


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();


        rate.sleep();
    }

    return 0;

}