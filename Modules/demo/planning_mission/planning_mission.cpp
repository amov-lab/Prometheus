//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>
#include "message_utils.h"

//topic 头文件
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/PositionReference.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
using namespace std;

#define MIN_DIS 0.1
# define NODE_NAME "planning_mission"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;

geometry_msgs::PoseStamped goal;                              // goal                    

prometheus_msgs::PositionReference fast_planner_cmd;          // fast planner cmd

bool sim_mode;
bool control_yaw_flag;
int flag_get_cmd = 0;
int flag_get_goal = 0;
float desired_yaw = 0;  //[rad]
float distance_to_goal = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Fast_planner();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void fast_planner_cmd_cb(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    flag_get_cmd = 1;
    fast_planner_cmd = *msg;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    distance_to_goal = sqrt(  pow(_DroneState.position[0] - goal.pose.position.x, 2) 
                            + pow(_DroneState.position[1] - goal.pose.position.y, 2) );
}
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    flag_get_goal = 1;
    cout << "Get a new goal!"<<endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_mission");
    ros::NodeHandle nh("~");

    nh.param<bool>("planning_mission/control_yaw_flag", control_yaw_flag, true);
    // 是否为仿真模式
    nh.param<bool>("planning_mission/sim_mode", sim_mode, false); 
    
    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    
    //【订阅】来自planning的指令
    ros::Subscriber fast_planner_sub   =    nh.subscribe<prometheus_msgs::PositionReference>("/prometheus/fast_planner/position_cmd", 50, fast_planner_cmd_cb);

    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10,goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);
   
    //　仿真模式下直接发送切换模式与起飞指令
    if(sim_mode == true)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Fast Planner<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please input 1 for start:"<<endl;
            cin >> start_flag;
        }
        // 起飞
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();
    }else
    {
        //　真实飞行情况：等待飞机状态变为offboard模式，然后发送起飞指令
        while(_DroneState.mode != "OFFBOARD")
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = 1 ;
            Command_Now.source = NODE_NAME;
            command_pub.publish(Command_Now);   
            cout << "Waiting for the offboard mode"<<endl;
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
    }

    while (ros::ok())
    {
        static int exec_num=0;
        exec_num++;

        // 若goal为99，则降落并退出任务
        if(goal.pose.position.x == 99)
        {
            // 抵达目标附近，则停止速度控制，改为位置控制
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Land;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;

            command_pub.publish(Command_Now);
            cout << "Quit... " << endl;

            return 0;
        }

        //回调
        ros::spinOnce();

        if( flag_get_cmd == 0)
        {
            if(exec_num == 10)
            {
                cout << "Waiting for trajectory" << endl;
                exec_num=0;
            }
            ros::Duration(0.5).sleep();
        }else if (distance_to_goal < MIN_DIS)
        {
            // 抵达目标附近，则停止速度控制，改为位置控制
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.position_ref[0]     = goal.pose.position.x;
            Command_Now.Reference_State.position_ref[1]     = goal.pose.position.y;
            Command_Now.Reference_State.position_ref[2]     = goal.pose.position.z;

            Command_Now.Reference_State.yaw_ref             = desired_yaw;
            command_pub.publish(Command_Now);

            if(exec_num == 10)
            {
                cout << "Arrived the goal, waiting for a new goal... " << endl;
                cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
                cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;
                exec_num=0;
            }

            flag_get_goal = 0;
            while (flag_get_goal == 0)
            {
                ros::spinOnce();
                ros::Duration(0.5).sleep();
            }
        }else
        {
            Fast_planner();
            ros::Duration(0.05).sleep();
        }
    }

    return 0;

}

void Fast_planner()
{
    if (control_yaw_flag)
    {
        // 根据速度大小决定是否更新期望偏航角， 更新采用平滑滤波的方式，系数可调
        // fastplanner航向策略仍然可以进一步优化
        if( sqrt( fast_planner_cmd.velocity_ref[1]* fast_planner_cmd.velocity_ref[1]
                 +  fast_planner_cmd.velocity_ref[0]* fast_planner_cmd.velocity_ref[0])  >  0.05  )
        {
            float next_desired_yaw_vel      = atan2(  fast_planner_cmd.velocity_ref[1] , 
                                                 fast_planner_cmd.velocity_ref[0]);
            float next_desired_yaw_pos      = atan2(  fast_planner_cmd.position_ref[1] - _DroneState.position[1],
                                                 fast_planner_cmd.position_ref[0] - _DroneState.position[0]);

            if(next_desired_yaw_pos > 0.8)
            {
                next_desired_yaw_pos = 0.8;
            }
            if(next_desired_yaw_pos < -0.8)
            {
                next_desired_yaw_pos = -0.8;
            }

            desired_yaw = (0.92*desired_yaw + 0.04*next_desired_yaw_pos + 0.04*next_desired_yaw_vel );
        }
    }else
    {
        desired_yaw = 0.0;
    }

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State =  fast_planner_cmd;
    Command_Now.Reference_State.yaw_ref = desired_yaw;

    command_pub.publish(Command_Now);
}