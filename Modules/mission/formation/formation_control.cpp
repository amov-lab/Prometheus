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
#include "command_to_mavros.h"
#include <unistd.h>

void formation::init()
{
    //控制命令数据订阅者
    cmd_sub = n.subscribe("/prometheus/control_command", 10, &formation::ControlCallBack, this);
    //集群队形数据订阅者
    formation_type_sub = n.subscribe("/prometheus/formation/change", 10, &formation::FormationChangeCallBack, this);

    //集群五台飞机位置控制数据发布者
    uav1_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
    uav2_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);
    uav3_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);
    uav4_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav4/mavros/setpoint_raw/local", 10);
    uav5_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav5/mavros/setpoint_raw/local", 10);

    //获取集群X轴间隔距离参数
    ros::param::param<double>("~FORMATION_DISTANCE_x", formation_distance_x, 1);

    //获取集群Y轴间隔距离参数
    ros::param::param<double>("~FORMATION_DISTANCE_y", formation_distance_y, 2);

    //获取定位来源
    ros::param::param<string>("Location_source", location_source, "fps");

    //获取飞控系统
    ros::param::param<string>("Flight_controller", flight_controller, "spm");

    //获取仿真中位置差值
    ros::param::param<double>("uav1_x", uav1_gazebo_offset_pose[0], 0);
    ros::param::param<double>("uav1_y", uav1_gazebo_offset_pose[1], 0);

    ros::param::param<double>("uav2_x", uav2_gazebo_offset_pose[0], 0);
    ros::param::param<double>("uav2_y", uav2_gazebo_offset_pose[1], 0);

    ros::param::param<double>("uav3_x", uav3_gazebo_offset_pose[0], 0);
    ros::param::param<double>("uav3_y", uav3_gazebo_offset_pose[1], 0);

    ros::param::param<double>("uav4_x", uav4_gazebo_offset_pose[0], 0);
    ros::param::param<double>("uav4_y", uav4_gazebo_offset_pose[1], 0);

    ros::param::param<double>("uav5_x", uav5_gazebo_offset_pose[0], 0);
    ros::param::param<double>("uav5_y", uav5_gazebo_offset_pose[1], 0);

    //获取是否为仿真参数
    ros::param::param<bool>("~sim",sim,false);

    check_param();

    //初始队形设置为一字形
    formation_data.type = prometheus_msgs::Formation::HORIZONTAL;

    //设置程序初始时间
    begin_time = ros::Time::now();
}

bool formation::check_param()
{
    char tmp[100];
    strncpy(tmp,location_source.c_str(),location_source.length() + 1);
    if((location_source == "gps" || location_source == "uwb") || location_source == "mocap")
    {   
        ROS_INFO("Location source is [%s]\n", tmp);           
    }
    else
    {
        ROS_ERROR("param [Location_source] value error, value is [%s], not gps, uwb or mocap\n", tmp);
        ros::shutdown();
    }
    
    strncpy(tmp,flight_controller.c_str(),flight_controller.length() + 1);
    if(flight_controller == "apm" || flight_controller == "px4")
    {   
        ROS_INFO("Flight_controller is [%s]\n", tmp);           
    }
    else
    {
        ROS_ERROR("param [flight_controller] value error, value is [%s], not apm or px4\n", tmp);
        ros::shutdown();
    }
}

//集群位置控制发布函数
void formation::formation_pos_pub()
{
    uav1_local_pub.publish(uav1_desired_pose);
    uav2_local_pub.publish(uav2_desired_pose);
    uav3_local_pub.publish(uav3_desired_pose);
    uav4_local_pub.publish(uav4_desired_pose);
    uav5_local_pub.publish(uav5_desired_pose);
}

//获取单台无人机控制数据
void formation::get_uav_cmd(Eigen::Vector3d offset_pose, mavros_msgs::PositionTarget& desired_pose)
{
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.type_mask = 0b100111111000;
    desired_pose.coordinate_frame = 1;
    desired_pose.position.x = control_data.Reference_State.position_ref[0] + offset_pose[0];
    desired_pose.position.y = control_data.Reference_State.position_ref[1] + offset_pose[1];
    desired_pose.position.z = control_data.Reference_State.position_ref[2] + offset_pose[2];
    desired_pose.yaw = control_data.Reference_State.yaw_ref;
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
    if(location_source == "mocap")
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
    else
    {
        if(location_source == "uwb" || location_source == "gps")
        {
            //1号机
            uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
            uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
            uav1_offset_pose[2] = 0;

            //2号机
            uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
            uav2_offset_pose[1] = 0- uav2_gazebo_offset_pose[1];
            uav2_offset_pose[2] = 0;

            //3号机
            uav3_offset_pose[0] = 0 - uav3_gazebo_offset_pose[0];
            uav3_offset_pose[1] = 0 - uav3_gazebo_offset_pose[1];
            uav3_offset_pose[2] = 0;

            //4号机
            uav4_offset_pose[0] = 0 - uav4_gazebo_offset_pose[0];
            uav4_offset_pose[1] = 0 - uav4_gazebo_offset_pose[1];
            uav4_offset_pose[2] = 0;

            //5号机
            uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
            uav5_offset_pose[1] = 0 - uav5_gazebo_offset_pose[1];
            uav5_offset_pose[2] = 0;
        }
    }
    

}

//设置三角队形
void formation::set_triangle()
{
    if(location_source == "mocap")
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
    if(location_source == "uwb" || location_source == "gps")
    {
        //1号机
        uav1_offset_pose[0] = 2 * formation_distance_x - uav1_gazebo_offset_pose[0];
        uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
        uav1_offset_pose[2] = 0;

        //2号机
        uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
        uav2_offset_pose[1] = 0 - uav2_gazebo_offset_pose[1];
        uav2_offset_pose[2] = 0;

        //3号机
        uav3_offset_pose[0] = formation_distance_x - uav3_gazebo_offset_pose[0];
        uav3_offset_pose[1] = 0 - uav3_gazebo_offset_pose[1];
        uav3_offset_pose[2] = 0;

        //4号机
        uav4_offset_pose[0] = formation_distance_x - uav4_gazebo_offset_pose[0];
        uav4_offset_pose[1] = 0 - uav4_gazebo_offset_pose[1];
        uav4_offset_pose[2] = 0;

        //5号机
        uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
        uav5_offset_pose[1] = 0 - uav5_gazebo_offset_pose[1];
        uav5_offset_pose[2] = 0;
    }
    
}

//设置菱形队形
void formation::set_diamond()
{
    if(location_source == "mocap")
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
    else
    {
        if(location_source == "gps" || location_source == "uwb")
        {
            ROS_WARN("gps or uwb location not support diamond formation");
        }
    }
    
    
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
    if(flight_controller == "px4")
    {
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
            formation_pos_pub();

            //等待0.1秒
            usleep(100000);
        }
    }
    else
    {
        if(flight_controller == "apm")
        {
            //起飞后开始控制无人机集群飞行
            ROS_INFO("When the drone formation takeoff and the control command input,enter 1 to start formation control");
            int check_flag;
            std::cin >> check_flag;
            if(check_flag != 1)
            {
                ros::shutdown();
            }

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
                }
                //五台无人机获取控制数据
                get_uav_cmd(uav1_offset_pose, uav1_desired_pose);
                get_uav_cmd(uav2_offset_pose, uav2_desired_pose);
                get_uav_cmd(uav3_offset_pose, uav3_desired_pose);
                get_uav_cmd(uav4_offset_pose, uav4_desired_pose);
                get_uav_cmd(uav5_offset_pose, uav5_desired_pose);
                formation_pos_pub();

                //等待0.1秒
                usleep(100000);
            }
        }
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