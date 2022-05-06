#include <ros/ros.h>
#include <iostream>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>

#include "math_utils.h"
#include "printf_utils.h"
using namespace std;
 
#define TIME_OUT 20.0
#define THRES_DISTANCE 0.15
#define NODE_NAME "waypoint_tracking"

prometheus_msgs::UAVState uav_state;                 // 无人机状态
prometheus_msgs::UAVCommand uav_command;             // 指令
Eigen::Vector3d uav_pos;
double distance_to_next_point;
Eigen::Vector3d waypoint[100];
int waypoint_num;
int waypoint_id = 1;

void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr& msg)
{
    uav_state = *msg;
    uav_pos = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
}

void printf_cb(const ros::TimerEvent& e)
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Waypoint Tracking Demo<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout <<"Waypoint tracking: [" << waypoint_id << "/ "<< waypoint_num << "]"<< endl;
    cout <<"Moving to point: " << waypoint[waypoint_id][0] << " [m] "<< waypoint[waypoint_id][1] << " [m] "<< waypoint[waypoint_id][2] << " [m] "<<endl;
    cout <<"Distance to waypoint: "<< distance_to_next_point << " [m]"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_tracking");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);

    // 默认使用1号机进行实验
    string uav_name = "/uav" + std::to_string(1); 
    //【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>(uav_name + "/prometheus/drone_state", 10, uav_state_cb);
    //【发布】发送给控制模块
    ros::Publisher uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>(uav_name + "/prometheus/control_command", 10);
    //【定时器】打印定时器
    ros::Timer printf_timer = nh.createTimer(ros::Duration(2.0), printf_cb); 

    nh.param("waypoint_num", waypoint_num, 5);
    
    cout <<"Total waypoint number: [ " << waypoint_num << " ]"<< endl;

    for(int i=1; i<=waypoint_num; i++)
    {
        nh.param("point"+std::to_string(i)+"_x", waypoint[i][0], 0.0);
        nh.param("point"+std::to_string(i)+"_y", waypoint[i][1], 0.0);
        nh.param("point"+std::to_string(i)+"_z", waypoint[i][2], 1.0);
        cout << "waypoint " << i << " : ["<<waypoint[i][0]<< "," <<waypoint[i][1]<<","<<waypoint[i][2]<<" ]"<<endl;
    }

    uav_command.Agent_CMD           = prometheus_msgs::UAVCommand::Current_Pos_Hover;
    uav_command_pub.publish(uav_command);

    int start_flag;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Waypoint Tracking Demo<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> start_flag;

    while(ros::ok)
    {
        distance_to_next_point = (waypoint[waypoint_id] - uav_pos).norm();

        if(waypoint_id == 5)
        {
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            uav_command_pub.publish(uav_command);
            break;
        }
        
        if(distance_to_next_point > THRES_DISTANCE)
        {            
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = waypoint[waypoint_id][0];
            uav_command.position_ref[1] = waypoint[waypoint_id][1];
            uav_command.position_ref[2] = waypoint[waypoint_id][2];
            uav_command.yaw_ref = 0.0;
            uav_command_pub.publish(uav_command);
        }else
        {
            waypoint_id++;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
