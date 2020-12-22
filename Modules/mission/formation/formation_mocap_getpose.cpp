/*******************************************************************
 * 文件名:formation_move.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.16
 * 
 * 介绍:该cpp文件主要为动捕集群中获取动捕系统位置数据并转发到飞控相关函数的实现以及程序的运行
 * ****************************************************************/

#include "Formation.h"
#include <unistd.h>

void formation::init()
{
    //动捕系统位置数据订阅者
    uav1_mocap_pose_sub = n.subscribe("/mocap/uav1/pose", 10, &formation::Uav1MocapPoseCallBack, this);
    uav2_mocap_pose_sub = n.subscribe("/mocap/uav2/pose", 10, &formation::Uav2MocapPoseCallBack, this);
    uav3_mocap_pose_sub = n.subscribe("/mocap/uav3/pose", 10, &formation::Uav3MocapPoseCallBack, this);
    uav4_mocap_pose_sub = n.subscribe("/mocap/uav4/pose", 10, &formation::Uav4MocapPoseCallBack, this);
    uav5_mocap_pose_sub = n.subscribe("/mocap/uav5/pose", 10, &formation::Uav5MocapPoseCallBack, this);
    //动捕系统位置数据发布者
    uav1_mocap_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/vision_pose/pose", 10);
    uav2_mocap_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/vision_pose/pose", 10);
    uav3_mocap_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/uav3/mavros/vision_pose/pose", 10);
    uav4_mocap_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/uav4/mavros/vision_pose/pose", 10);
    uav5_mocap_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/uav5/mavros/vision_pose/pose", 10);
}

void formation::getpose()
{
    //初始化
    init();
    while(ros::ok())
    {
        //处理回调函数,
        ros::spinOnce();
        //打上时间戳后,发布位置数据到飞控
        uav1_current_pose.header.stamp = ros::Time::now();
        uav2_current_pose.header.stamp = ros::Time::now();
        uav3_current_pose.header.stamp = ros::Time::now();
        uav4_current_pose.header.stamp = ros::Time::now();
        uav5_current_pose.header.stamp = ros::Time::now();
        uav1_mocap_pose_pub.publish(uav1_current_pose);
        uav2_mocap_pose_pub.publish(uav2_current_pose);
        uav3_mocap_pose_pub.publish(uav3_current_pose);
        uav4_mocap_pose_pub.publish(uav4_current_pose);
        uav5_mocap_pose_pub.publish(uav5_current_pose);
        usleep(10000);
    }
}

//获取动捕系统下的五台无人机位置数据

//1号机
void formation::Uav1MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav1_current_pose.pose = pose_msgs->pose;
}

//2号机
void formation::Uav2MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav2_current_pose.pose = pose_msgs->pose;
}

//3号机
void formation::Uav3MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav3_current_pose.pose = pose_msgs->pose;
}

//4号机
void formation::Uav4MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav4_current_pose.pose = pose_msgs->pose;
}

//5号机
void formation::Uav5MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav5_current_pose.pose = pose_msgs->pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation/mocap/get_pose");
    formation Formation;
    Formation.getpose();
    return 0;
}