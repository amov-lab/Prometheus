//头文件
#include <ros/ros.h>
#include <prometheus_control_utils.h>


//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;
//---------------------------------------相关参数-----------------------------------------------

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void msg_main_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    prometheus_control_utils::printf_message(message);
}
void msg_local_planner_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    prometheus_control_utils::printf_message(message);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station_msg");
    ros::NodeHandle nh("~");

    // 【订阅】 各类消息
    ros::Subscriber message_main_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/main", 10, msg_main_cb);
    ros::Subscriber message_local_planner_sub = nh.subscribe<prometheus_msgs::Message>("/prometheus/message/local_planner", 10, msg_local_planner_cb);

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