//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/PositionReference.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include "message_utils.h"

using namespace std;

# define NODE_NAME "formation_flight"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int uav_number;

prometheus_msgs::SwarmCommand Command_uav1;  
prometheus_msgs::SwarmCommand Command_uav2;  
prometheus_msgs::SwarmCommand Command_uav3;  
prometheus_msgs::SwarmCommand Command_uav4;  
prometheus_msgs::SwarmCommand Command_uav5;  

Eigen::Vector3f virtual_leader_pos;
float virtual_leader_yaw;
float formation_size;

ros::Publisher uav1_command_pub;
ros::Publisher uav2_command_pub;
ros::Publisher uav3_command_pub;
ros::Publisher uav4_command_pub;
ros::Publisher uav5_command_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void triangle();
void one_column();
void triangle_to_one_column();
void one_column_to_triangle();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cout << "Get a new goal from rviz!"<<endl;
    virtual_leader_pos[0] = msg->pose.position.x;
    virtual_leader_pos[1] = msg->pose.position.y;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_flight");
    ros::NodeHandle nh("~");

    nh.param<int>("uav_number", uav_number, 5);
    nh.param<float>("virtual_leader_pos_x", virtual_leader_pos[0], 0.0);
    nh.param<float>("virtual_leader_pos_y", virtual_leader_pos[1], 0.0);
    nh.param<float>("virtual_leader_pos_z", virtual_leader_pos[2], 1.0);
    nh.param<float>("virtual_leader_yaw", virtual_leader_yaw, 0.0);
    nh.param<float>("formation_size", formation_size, 1.0);

    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/formation/virtual_leader", 10, goal_cb);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    //【发布】阵型
    uav1_command_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/uav1/prometheus/swarm_command", 10);
    uav2_command_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/uav2/prometheus/swarm_command", 10);
    uav3_command_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/uav3/prometheus/swarm_command", 10);
    uav4_command_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/uav4/prometheus/swarm_command", 10);
    uav5_command_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/uav5/prometheus/swarm_command", 10);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;

    ros::Duration(1.0).sleep();

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm and takeoff all the UAVs."<<endl;
        cin >> start_flag;

        Command_uav1.Mode = prometheus_msgs::SwarmCommand::Idle;
        Command_uav1.yaw_ref = 999;

        uav1_command_pub.publish(Command_uav1);
        uav2_command_pub.publish(Command_uav1);
        uav3_command_pub.publish(Command_uav1);
        uav4_command_pub.publish(Command_uav1);
        uav5_command_pub.publish(Command_uav1);
    }

    start_flag = 0;

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm and takeoff all the UAVs."<<endl;
        cin >> start_flag;

        Command_uav1.Mode = prometheus_msgs::SwarmCommand::Takeoff;
        Command_uav1.yaw_ref = 0.0;

        uav1_command_pub.publish(Command_uav1);
        uav2_command_pub.publish(Command_uav1);
        uav3_command_pub.publish(Command_uav1);
        uav4_command_pub.publish(Command_uav1);
        uav5_command_pub.publish(Command_uav1);
    }

    float x_sp,y_sp;
    int formation_num = 1;
    float trajectory_total_time;
    while (ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the mission: 1 for formation change, 2 for Point Tracking, 3 for Circle Trajectory Tracking..."<<endl;
        cin >> start_flag;

        if (start_flag == 1)
        {
            if(formation_num == 1)
            {
                cout << "Changing formation to Triangle..."<<endl;
                triangle();
                formation_num = 2;
            }else
            {
                cout << "Changing formation to One Column..."<<endl;
                one_column();
                formation_num = 1;
            }
        }else if (start_flag == 2)
        {
            cout << "Please enter the desired position:"<<endl;
            cout << "virtual_leader_pos: --- x [m] "<<endl;
            cin >> virtual_leader_pos[0];
            cout << "virtual_leader_pos: --- y [m]"<<endl;
            cin >> virtual_leader_pos[1];
            cout << "virtual_leader_yaw [deg]:"<<endl;
            cin >> virtual_leader_yaw;
            virtual_leader_yaw = virtual_leader_yaw/180.0*M_PI;

            if(formation_num == 2)
            {
                triangle();
            }else if(formation_num == 1)
            {
                one_column();
            }
            
        }else if (start_flag == 3)
        {
            cout << "Input the trajectory_total_time:"<<endl;
            cin >> trajectory_total_time;

            float time_trajectory = 0.0;

            while(time_trajectory < trajectory_total_time)
            {

                const float omega = 1.0;
                const float circle_radius = 1.0;

                virtual_leader_pos[0] = virtual_leader_pos[0] + circle_radius * cos(time_trajectory * omega);
                virtual_leader_pos[1] = virtual_leader_pos[1] + circle_radius * sin(time_trajectory * omega);

                time_trajectory = time_trajectory + 0.01;

                cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

                if(formation_num == 2)
                {
                    triangle();
                }else if(formation_num == 1)
                {
                    one_column();
                }

                ros::Duration(0.01).sleep();
            }
        }else
        {
            cout << "Wrong input."<<endl;
        }
        
        cout << "virtual_leader_pos [X Y] : " << virtual_leader_pos[0] << " [ m ] "<< virtual_leader_pos[1] <<" [ m ] "<< endl;
        cout << "virtual_leader_yaw: " << virtual_leader_yaw/M_PI*180.0 <<" [ deg ] "<< endl;
     
        ros::Duration(2.0).sleep();
    }

    return 0;

}

void one_column()
{   
    Command_uav1.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav1.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav1.position_ref[1] = virtual_leader_pos[1] + 2 * formation_size;
    Command_uav1.position_ref[2] = virtual_leader_pos[2] ;  
    Command_uav1.yaw_ref = virtual_leader_yaw;
    uav1_command_pub.publish(Command_uav1);

    Command_uav2.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav2.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav2.position_ref[1] = virtual_leader_pos[1] + 1 * formation_size; 
    Command_uav2.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav2.yaw_ref = virtual_leader_yaw;
    uav2_command_pub.publish(Command_uav2);

    Command_uav3.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav3.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav3.position_ref[1] = virtual_leader_pos[1] ; 
    Command_uav3.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav3.yaw_ref = virtual_leader_yaw;
    uav3_command_pub.publish(Command_uav3);

    Command_uav4.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav4.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav4.position_ref[1] = virtual_leader_pos[1] - 1 * formation_size; 
    Command_uav4.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav4.yaw_ref = virtual_leader_yaw;
    uav4_command_pub.publish(Command_uav4);

    Command_uav5.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav5.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav5.position_ref[1] = virtual_leader_pos[1] - 2 * formation_size; 
    Command_uav5.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav5.yaw_ref = virtual_leader_yaw;
    uav5_command_pub.publish(Command_uav5);
}

void triangle()
{   
    //Velocity_Control
    Command_uav1.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav1.position_ref[0] = virtual_leader_pos[0] - formation_size; 
    Command_uav1.position_ref[1] = virtual_leader_pos[1] + formation_size; 
    Command_uav1.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav1.yaw_ref = virtual_leader_yaw;
    uav1_command_pub.publish(Command_uav1);

    Command_uav2.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav2.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav2.position_ref[1] = virtual_leader_pos[1] + 0.5 * formation_size; 
    Command_uav2.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav2.yaw_ref = virtual_leader_yaw;
    uav2_command_pub.publish(Command_uav2);

    Command_uav3.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav3.position_ref[0] = virtual_leader_pos[0] + formation_size; 
    Command_uav3.position_ref[1] = virtual_leader_pos[1] ; 
    Command_uav3.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav3.yaw_ref = virtual_leader_yaw;
    uav3_command_pub.publish(Command_uav3);

    Command_uav4.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav4.position_ref[0] = virtual_leader_pos[0] ; 
    Command_uav4.position_ref[1] = virtual_leader_pos[1] - 0.5 * formation_size; 
    Command_uav4.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav4.yaw_ref = virtual_leader_yaw;
    uav4_command_pub.publish(Command_uav4);

    Command_uav5.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    Command_uav5.position_ref[0] = virtual_leader_pos[0] - formation_size; 
    Command_uav5.position_ref[1] = virtual_leader_pos[1] - formation_size; 
    Command_uav5.position_ref[2] = virtual_leader_pos[2] ; 
    Command_uav5.yaw_ref = virtual_leader_yaw;
    uav5_command_pub.publish(Command_uav5);
}