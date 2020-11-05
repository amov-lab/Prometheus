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
int controller_num;

prometheus_msgs::SwarmCommand swarm_command;  

Eigen::Vector3f virtual_leader_pos;
Eigen::Vector3f virtual_leader_vel;
float virtual_leader_yaw;
float formation_size;
int formation_num = 1;

ros::Publisher uav1_command_pub;
ros::Publisher uav2_command_pub;
ros::Publisher uav3_command_pub;
ros::Publisher uav4_command_pub;
ros::Publisher uav5_command_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();
void pub_formation_command();
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
    
    // 0代表位置追踪模式，１代表速度追踪模式，２代表加速度追踪模式 
    nh.param<int>("controller_num", controller_num, 0);
    nh.param<float>("virtual_leader_pos_x", virtual_leader_pos[0], 0.0);
    nh.param<float>("virtual_leader_pos_y", virtual_leader_pos[1], 0.0);
    nh.param<float>("virtual_leader_pos_z", virtual_leader_pos[2], 0.6);
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
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;

    printf_param();

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm all the UAVs."<<endl;
        cin >> start_flag;

        swarm_command.Mode = prometheus_msgs::SwarmCommand::Idle;
        swarm_command.yaw_ref = 999;

        uav1_command_pub.publish(swarm_command);
        uav2_command_pub.publish(swarm_command);
        uav3_command_pub.publish(swarm_command);
        uav4_command_pub.publish(swarm_command);
        uav5_command_pub.publish(swarm_command);
    }

    start_flag = 0;

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;

        swarm_command.Mode = prometheus_msgs::SwarmCommand::Takeoff;
        swarm_command.yaw_ref = 0.0;

        uav1_command_pub.publish(swarm_command);
        uav2_command_pub.publish(swarm_command);
        uav3_command_pub.publish(swarm_command);
        uav4_command_pub.publish(swarm_command);
        uav5_command_pub.publish(swarm_command);
    }

    
    float trajectory_total_time;
    while (ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< 3 for Circle Trajectory Tracking,"<< endl;
        cout << "Please choose the action: 0 for Formation Shape, 1 for Virtual Leader Pos, 2 for Hold, 3 for Land, 4 for Circle, 5 for Disarm..."<<endl;
        cin >> start_flag;

        if (start_flag == 0)
        {
            cout << "Please choose the formation: 1 for One Column, 2 for Triangle, 3 for One Row ..."<<endl;
            cin >> formation_num;

            pub_formation_command();
            
        }else if (start_flag == 1)
        {
            cout << "Please enter the virtual leader position:"<<endl;
            cout << "virtual_leader_pos: --- x [m] "<<endl;
            cin >> virtual_leader_pos[0];
            cout << "virtual_leader_pos: --- y [m]"<<endl;
            cin >> virtual_leader_pos[1];
            cout << "virtual_leader_pos: --- z [m]"<<endl;
            cin >> virtual_leader_pos[2];
            cout << "virtual_leader_yaw [deg]:"<<endl;
            cin >> virtual_leader_yaw;
            virtual_leader_yaw = virtual_leader_yaw/180.0*M_PI;

            pub_formation_command();
            
        }else if (start_flag == 2)
        {
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Hold;

            uav1_command_pub.publish(swarm_command);
            uav2_command_pub.publish(swarm_command);
            uav3_command_pub.publish(swarm_command);
            uav4_command_pub.publish(swarm_command);
            uav5_command_pub.publish(swarm_command);
        }else if (start_flag == 3)
        {
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Land;

            uav1_command_pub.publish(swarm_command);
            uav2_command_pub.publish(swarm_command);
            uav3_command_pub.publish(swarm_command);
            uav4_command_pub.publish(swarm_command);
            uav5_command_pub.publish(swarm_command);
        }else if (start_flag == 4)
        {
            cout << "Input the trajectory_total_time:"<<endl;
            cin >> trajectory_total_time;

            float time_trajectory = 0.0;

            while(time_trajectory < trajectory_total_time)
            {

                const float omega = 0.15;
                const float circle_radius = 1.5;

                virtual_leader_pos[0] = circle_radius * cos(time_trajectory * omega);
                virtual_leader_pos[1] = circle_radius * sin(time_trajectory * omega);
                //virtual_leader_pos[2] = 1.0;
                virtual_leader_vel[0] = - omega * circle_radius * sin(time_trajectory * omega);
                virtual_leader_vel[1] = omega * circle_radius * cos(time_trajectory * omega);
                virtual_leader_vel[2] = 0.0;

                time_trajectory = time_trajectory + 0.01;

                cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

                pub_formation_command();
                ros::Duration(0.01).sleep();
            }
        }else if (start_flag == 5)
        {
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Disarm;

            uav1_command_pub.publish(swarm_command);
            uav2_command_pub.publish(swarm_command);
            uav3_command_pub.publish(swarm_command);
            uav4_command_pub.publish(swarm_command);
            uav5_command_pub.publish(swarm_command);
        }else
        {
            cout << "Wrong input."<<endl;
        }
        
        cout << "virtual_leader_pos [X Y] : " << virtual_leader_pos[0] << " [ m ] "<< virtual_leader_pos[1] <<" [ m ] "<< virtual_leader_pos[2] <<" [ m ] "<< endl;
        cout << "virtual_leader_yaw: " << virtual_leader_yaw/M_PI*180.0 <<" [ deg ] "<< endl;
     
        ros::Duration(2.0).sleep();
    }

    return 0;

}

void pub_formation_command()
{
    if(formation_num == 1)
    {
        cout << "Formation shape: [ One_column ]"<<endl;

        swarm_command.swarm_shape = prometheus_msgs::SwarmCommand::One_column;
    }else if(formation_num == 2)
    {
        cout << "Formation shape: [ Triangle ]"<<endl;

        swarm_command.swarm_shape = prometheus_msgs::SwarmCommand::Triangle;
    }else if(formation_num == 3)
    {
        cout << "Formation shape: [ One_row ]"<<endl;

        swarm_command.swarm_shape = prometheus_msgs::SwarmCommand::One_row;
    }else
    {
        cout << "Wrong formation shape!"<<endl;
    }

    if (controller_num == 0)
    {
        swarm_command.Mode = prometheus_msgs::SwarmCommand::Position_Control;
    }else if(controller_num == 1)
    {
        swarm_command.Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
    }else if(controller_num == 2)
    {
        swarm_command.Mode = prometheus_msgs::SwarmCommand::Accel_Control;
    }

    swarm_command.swarm_size = formation_size;
    swarm_command.position_ref[0] = virtual_leader_pos[0] ; 
    swarm_command.position_ref[1] = virtual_leader_pos[1] ;
    swarm_command.position_ref[2] = virtual_leader_pos[2] ;  
    swarm_command.velocity_ref[0] = virtual_leader_vel[0] ; 
    swarm_command.velocity_ref[1] = virtual_leader_vel[1] ; 
    swarm_command.velocity_ref[2] = virtual_leader_vel[2] ; 
    swarm_command.yaw_ref = virtual_leader_yaw;

    uav1_command_pub.publish(swarm_command);
    uav2_command_pub.publish(swarm_command);
    uav3_command_pub.publish(swarm_command);
    uav4_command_pub.publish(swarm_command);
    uav5_command_pub.publish(swarm_command);

}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "controller_num   : "<< controller_num <<endl;
    
    cout << "virtual_leader_pos_x   : "<< virtual_leader_pos[0]<<" [m] "<<endl;
    cout << "virtual_leader_pos_y   : "<< virtual_leader_pos[1]<<" [m] "<<endl;
    cout << "virtual_leader_pos_z   : "<< virtual_leader_pos[2]<<" [m] "<<endl;
    cout << "virtual_leader_yaw : "<< virtual_leader_yaw << " [rad]  "<< endl;
    cout << "formation_size : "<< formation_size << " [m] "<< endl;
}