#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/DroneState.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

using namespace std;

#define TRA_WINDOW 1000

// 无人机初始位置
double  init_pos[3];
// 规划器类型
int planner_type;

prometheus_msgs::DroneState _DroneState;
ros::Publisher drone_state_pub,odom_pub,trajectory_pub,meshPub;

bool start_flag;

prometheus_msgs::PositionReference traj_now;

std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

static  string mesh_resource;
static double color_r, color_g, color_b, color_a, scale;

void uav_pos_pub() ;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 【订阅】订阅轨迹，并赋值给odom，实现近似动态
void planner_cmd_cb(const prometheus_msgs::ControlCommand& msg)
{
    prometheus_msgs::ControlCommand command_now;
    command_now = msg; 

    if(command_now.Mode   == prometheus_msgs::ControlCommand::Move)
    {
        start_flag = true;

        _DroneState.connected = true;
        _DroneState.armed = true;
        _DroneState.landed = false;
        _DroneState.mode = "OFFBOARD";

        _DroneState.position[0] = command_now.Reference_State.position_ref[0];
        _DroneState.position[1] = command_now.Reference_State.position_ref[1];
        _DroneState.position[2] = command_now.Reference_State.position_ref[2];

        _DroneState.velocity[0] = command_now.Reference_State.velocity_ref[0];
        _DroneState.velocity[1] = command_now.Reference_State.velocity_ref[1];
        _DroneState.velocity[2] = command_now.Reference_State.velocity_ref[2];
    }
}
int main(int argc,char** argv)
{   
    // 1. initialization node
    ros::init(argc,argv,"uav_sim");
    ros::NodeHandle nh("~");

    nh.param("init_pos_x", init_pos[0], 0.0);
    nh.param("init_pos_y", init_pos[1], 0.0);
    nh.param("init_pos_z", init_pos[2], 1.0);
    nh.param("planner_type", planner_type, 0);
    nh.param("mesh_resource", mesh_resource, std::string("package://prometheus_planning_sim/meshes/hummingbird.mesh"));
    nh.param("color/r", color_r, 1.0);
    nh.param("color/g", color_g, 0.0);
    nh.param("color/b", color_b, 0.0);
    nh.param("color/a", color_a, 1.0);
    nh.param("robot_scale", scale, 2.0);   

    // 订阅规划器指令
    ros::Subscriber planner_cmd_sub = nh.subscribe("/prometheus/control_command", 50, planner_cmd_cb);

    // 发布无人机状态
    drone_state_pub = nh.advertise<prometheus_msgs::DroneState>("/prometheus/drone_state", 10);

    //【发布】无人机odometry，用于RVIZ显示
    odom_pub = nh.advertise<nav_msgs::Odometry>("/prometheus/drone_odom", 10);

    meshPub   = nh.advertise<visualization_msgs::Marker>("/prometheus/robot_marker",   100);  

    // 【发布】无人机移动轨迹，用于RVIZ显示
    trajectory_pub = nh.advertise<nav_msgs::Path>("/prometheus/drone_trajectory", 10);

    start_flag = false;

    ROS_INFO(" the simulator initialization successful!");

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        uav_pos_pub();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

// 【发布】处理里程计信息，根据模式选择是否发布
void uav_pos_pub() 
{
    // 未开始任务 发布初始位置
    if (start_flag==false)
    {
        _DroneState.connected = true;
        _DroneState.armed = true;
        _DroneState.landed = false;
        _DroneState.mode = "OFFBOARD";

        _DroneState.position[0] = init_pos[0];
        _DroneState.position[1] = init_pos[1];
        _DroneState.position[2] = init_pos[2];

        _DroneState.velocity[0] = 0.0;
        _DroneState.velocity[1] = 0.0;
        _DroneState.velocity[2] = 0.0;

        geometry_msgs::Quaternion uav_quat;
        uav_quat.x = 0;
        uav_quat.y = 0;
        uav_quat.z = 0;
        uav_quat.w = 0;
        _DroneState.attitude_q = uav_quat;
        
        drone_state_pub.publish(_DroneState);
    }
    else if (start_flag==true) 
    {
        // should have some dynamics! 
        drone_state_pub.publish(_DroneState);
    }

    // 发布无人机当前odometry,用于导航及rviz显示
    nav_msgs::Odometry Drone_odom;
    Drone_odom.header.stamp = ros::Time::now();
    Drone_odom.header.frame_id = "world";
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];

    // 导航算法规定 高度不能小于0
    if (Drone_odom.pose.pose.position.z <= 0)
    {
        Drone_odom.pose.pose.position.z = 0.01;
    }

    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];
    odom_pub.publish(Drone_odom);



    // 发布mesh
    // Mesh model             
    visualization_msgs::Marker meshROS;                                     
    meshROS.header.frame_id = "world";
    meshROS.header.stamp =  ros::Time::now();
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = _DroneState.position[0];
    meshROS.pose.position.y = _DroneState.position[1];
    meshROS.pose.position.z = _DroneState.position[2];
    meshROS.pose.orientation.w = 1.0;
    meshROS.pose.orientation.x = 0.0;
    meshROS.pose.orientation.y = 0.0;
    meshROS.pose.orientation.z = 0.0;
    meshROS.scale.z = scale;
    meshROS.color.a = color_a;
    meshROS.color.r = color_r;
    meshROS.color.g = color_g;
    meshROS.color.b = color_b;
    meshROS.text = "UAV";
    // meshROS.mesh_resource = mesh_resource;
    // meshROS.mesh_use_embedded_materials = true;
    meshPub.publish(meshROS);         

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped drone_pos;
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "world";
    drone_pos.pose.position.x = _DroneState.position[0];
    drone_pos.pose.position.y = _DroneState.position[1];
    drone_pos.pose.position.z = _DroneState.position[2];

    drone_pos.pose.orientation = _DroneState.attitude_q;

    //发布无人机的位姿 和 轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), drone_pos);
    if (posehistory_vector_.size() > TRA_WINDOW)
    {
        posehistory_vector_.pop_back();
    }

    nav_msgs::Path drone_trajectory;
    drone_trajectory.header.stamp = ros::Time::now();
    drone_trajectory.header.frame_id = "world";
    drone_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(drone_trajectory);
}


