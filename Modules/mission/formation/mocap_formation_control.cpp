/*******************************************************************
 * 文件名:mocap_formation_control.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.14
 * 
 * 介绍:该cpp文件主要为动捕集群中集群控制相关函数的实现以及程序的运行
 * ****************************************************************/
#include "Formation.h"
#include <unistd.h>

void formation::init()
{
    //控制命令数据订阅者
    cmd_sub = n.subscribe("/prometheus/control_command", 10, &formation::ControlCallBack, this);
    //集群队形数据订阅者
    formation_type_sub = n.subscribe("/prometheus/formation/change", 10, &formation::FormationChangeCallBack, this);

    //获取集群X轴间隔距离参数
    ros::param::param<double>("~FORMATION_DISTANCE_x", formation_distance_x, 1);

    //获取集群Y轴间隔距离参数
    ros::param::param<double>("~FORMATION_DISTANCE_y", formation_distance_y, 2);

    //获取仿真中位置差值
    ros::param::param<double>("~uav1_x", uav1_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav1_y", uav1_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav2_x", uav2_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav2_y", uav2_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav3_x", uav3_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav3_y", uav3_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav4_x", uav4_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav4_y", uav4_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav5_x", uav5_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav5_y", uav5_gazebo_offset_pose[1], 0);

    //获取是否为仿真参数
    ros::param::param<bool>("~sim",sim,false);

    //初始队形设置为一字形
    formation_data.type = prometheus_msgs::Formation::HORIZONTAL;

    //设置程序初始时间
    begin_time = ros::Time::now();
}

void formation::is_sim()
{
    if(!sim)
    {
        uav1_gazebo_offset_pose[0] = 0;
        uav1_gazebo_offset_pose[1] = 0;

        uav2_gazebo_offset_pose[0] = 0;
        uav2_gazebo_offset_pose[1] = 0;

        uav3_gazebo_offset_pose[0] = 0;
        uav3_gazebo_offset_pose[1] = 0;

        uav4_gazebo_offset_pose[0] = 0;
        uav4_gazebo_offset_pose[1] = 0;

        uav5_gazebo_offset_pose[0] = 0;
        uav5_gazebo_offset_pose[1] = 0;
    }
}

//设置一字队形
void formation::set_horizontal()
{
    //1号机
    uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = 0 - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = formation_distance_y - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = 0 - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = -formation_distance_y - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;

}

//设置三角队形
void formation::set_triangle()
{
    //1号机
    uav1_offset_pose[0] = 2 * formation_distance_x - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = formation_distance_x - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = formation_distance_y - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = formation_distance_x - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = -formation_distance_y - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;
}

//设置菱形队形
void formation::set_diamond()
{
    //1号机
    uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = 2 * formation_distance_x - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = 0 - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = -2 * formation_distance_x - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = 0 - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;
}

//设置菱形队形相关过渡队形
void formation::set_diamond_stage1()
{
    //1号机
    uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = 2 * formation_distance_x - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = formation_distance_y - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = -2 * formation_distance_x - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = -formation_distance_y - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;
}

//集群控制函数
void formation::control()
{
    //初始化
    init();
    is_sim();
    while(ros::ok())
    {
        //处理回调函数
        ros::spinOnce();
        //获取集群队形
        switch(formation_data.type)
        {
            //设置为一字队形
            case prometheus_msgs::Formation::HORIZONTAL:
                set_horizontal();
                break;
        
            //设置为三角队形
            case prometheus_msgs::Formation::TRIANGEL:
                set_triangle();
                break;

            //设置为菱形队形过渡队形
            case prometheus_msgs::Formation::DIAMOND_STAGE_1:
                set_diamond_stage1();
                break;

            //设置为菱形队形
            case prometheus_msgs::Formation::DIAMOND:
                set_diamond();
                break;
        }
        //五台无人机获取控制数据
        get_uav_cmd(uav1_offset_pose, uav1_desired_pose);
        get_uav_cmd(uav2_offset_pose, uav2_desired_pose);
        get_uav_cmd(uav3_offset_pose, uav3_desired_pose);
        get_uav_cmd(uav4_offset_pose, uav4_desired_pose);
        get_uav_cmd(uav5_offset_pose, uav5_desired_pose);
        //等待0.1秒
        usleep(100000);
    }
    

}

//获取无人机控制指令
void formation::ControlCallBack(const prometheus_msgs::ControlCommandConstPtr& control_msgs)
{
    control_data = *control_msgs;
}

//获取无人机集群队形指令
void formation::FormationChangeCallBack(const prometheus_msgs::FormationConstPtr& change_msgs)
{
    formation_data.type = change_msgs->type;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_control");
    formation Formation;
    Formation.control();
    return 0;
}