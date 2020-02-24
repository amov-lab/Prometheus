//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/ControlCommand.h>
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
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
geometry_msgs::Point Desired_vel;


prometheus_msgs::PositionReference pos_cmd;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;
ros::Publisher drone_pub;
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;
int flag_get_cmd = 0;
geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped drone_pos;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int posehistory_window_ = 1000;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void desired_vel_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    flag_get_cmd = 1;
    Desired_vel = *msg;

    float desired_yaw;  //[rad]

    desired_yaw = atan2(Desired_vel.y, Desired_vel.x);

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_VEL;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.velocity_ref[0]     = Desired_vel.x;
    Command_Now.Reference_State.velocity_ref[1]     = Desired_vel.y;
    Command_Now.Reference_State.velocity_ref[2]     = Desired_vel.z;
    Command_Now.Reference_State.yaw_ref             = desired_yaw;

    command_pub.publish(Command_Now);
    cout << "desired_yaw: " << desired_yaw / M_PI * 180 << " [deg] "<<endl;
    cout << "desired_vel: " << Desired_vel.x << " [m/s] "<< Desired_vel.y << " [m/s] "<< Desired_vel.z << " [m/s] "<<endl;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    nav_msgs::Odometry Odom_Now;
    Odom_Now.header.stamp = ros::Time::now();
    Odom_Now.header.frame_id = "world";

    Odom_Now.pose.pose.position.x = _DroneState.position[0];
    Odom_Now.pose.pose.position.y = _DroneState.position[1];
    Odom_Now.pose.pose.position.z = _DroneState.position[2];

    if(Odom_Now.pose.pose.position.z <= 0)
    {
        Odom_Now.pose.pose.position.z = 0.01;
    }

//    Odom_Now.pose.pose.orientation = geometry_msgs::Quaternion(_DroneState.attitude[2],_DroneState.attitude[1], _DroneState.attitude[0]); // yaw, pitch, roll
    Odom_Now.pose.pose.orientation = _DroneState.attitude_q;
    Odom_Now.twist.twist.linear.x = _DroneState.velocity[0];
    Odom_Now.twist.twist.linear.y = _DroneState.velocity[1];
    Odom_Now.twist.twist.linear.z = _DroneState.velocity[2];
    odom_pub.publish(Odom_Now);

    
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "world";
    drone_pos.pose.position.x = _DroneState.position[0];
    drone_pos.pose.position.y = _DroneState.position[1];
    drone_pos.pose.position.z = _DroneState.position[2];

    drone_pos.pose.orientation = _DroneState.attitude_q;
    drone_pub.publish(drone_pos);

    //发布无人机的位姿 和 轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), drone_pos);
    if(posehistory_vector_.size() > posehistory_window_){
        posehistory_vector_.pop_back();
    }
    
    nav_msgs::Path drone_trajectory;
    drone_trajectory.header.stamp = ros::Time::now();
    drone_trajectory.header.frame_id = "world";
    drone_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(drone_trajectory);

}



void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh("~");
    ros::Subscriber local_planner_sub =nh.subscribe<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 50, desired_vel_cb);
    
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/planning/goal", 10,goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    
    //【发布】无人机odometry
    odom_pub = nh.advertise<nav_msgs::Odometry>("/planning/odom_world", 10);

    trajectory_pub = nh.advertise<nav_msgs::Path>("/planning/drone_trajectory", 10);
    drone_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/drone_pose", 10);

    // 频率 [20Hz]
    ros::Rate rate(20.0);

    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;
    command_pub.publish(Command_Now);

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

    
    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        if( flag_get_cmd == 0)
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Hold;
            Command_Now.Command_ID                          = 1;

            command_pub.publish(Command_Now);
            cout << "waiting for trajectory" << endl;
        }
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Planning<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "drone_pos: " << drone_pos.pose.position.x << " [m] "<< drone_pos.pose.position.y << " [m] "<< drone_pos.pose.position.z << " [m] "<<endl;
        cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;

        


        rate.sleep();
    }

    return 0;

}
