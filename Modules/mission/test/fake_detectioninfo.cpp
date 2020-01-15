//ROS 头文件
#include <ros/ros.h>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/DetectionInfo.h>

using namespace std;

prometheus_msgs::DetectionInfo Detection_Info;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_detectioninfo");
    ros::NodeHandle nh("~");

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher detection_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/prometheus/target", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    while(ros::ok())
    {
        Detection_Info.header.stamp                    = ros::Time::now();
        cout << "Please input the detection info: "<<endl;
        cout << "position[0]: "<<endl;
        cin >> Detection_Info.position[0];
        cout << "position[1]: "<<endl;
        cin >> Detection_Info.position[1];
        cout << "position[2]: "<<endl;
        cin >> Detection_Info.position[2];

        cout << "attitude[2]: "<<endl;
        cin >> Detection_Info.attitude[2];
        Detection_Info.detected = true;
        Detection_Info.yaw_error = Detection_Info.attitude[2];

        detection_pub.publish(Detection_Info);
    }

    return 0;

}