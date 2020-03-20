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

#define MIN_DIS 0.1
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
geometry_msgs::Point Desired_vel;


prometheus_msgs::PositionReference pos_cmd;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令

prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;
int flag_get_cmd = 0;
geometry_msgs::PoseStamped goal;

float desired_yaw = 0;  //[rad]
float distance_to_goal = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void desired_vel_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    flag_get_cmd = 1;
    Desired_vel = *msg;

    distance_to_goal = sqrt(  pow(_DroneState.position[0] - goal.pose.position.x, 2) 
                            + pow(_DroneState.position[1] - goal.pose.position.y, 2) );

    // 抵达目标附近，则停止速度控制，改为位置控制
    if (distance_to_goal < MIN_DIS)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = goal.pose.position.x;
        Command_Now.Reference_State.position_ref[1]     = goal.pose.position.y;
        Command_Now.Reference_State.position_ref[2]     = 1.5;
        Command_Now.Reference_State.yaw_ref             = desired_yaw;
    }else
    {
        //根据速度大小决定是否更新期望偏航角， 更新采用平滑滤波的方式，系数可调
        if( sqrt(Desired_vel.x*Desired_vel.x + Desired_vel.y*Desired_vel.y)  >  0.2   )
        {
            desired_yaw = (0.6*desired_yaw + 0.4*atan2(Desired_vel.y, Desired_vel.x) );
        }
        
        //高度改为定高飞行
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.velocity_ref[0]     = Desired_vel.x;
        Command_Now.Reference_State.velocity_ref[1]     = Desired_vel.y;
        Command_Now.Reference_State.position_ref[2]     = 1.5;
        Command_Now.Reference_State.yaw_ref             = desired_yaw;
    }

    command_pub.publish(Command_Now);
    cout << "desired_yaw: " << desired_yaw / M_PI * 180 << " [deg] "<<endl;
    cout << "desired_vel: " << Desired_vel.x << " [m/s] "<< Desired_vel.y << " [m/s] "<< Desired_vel.z << " [m/s] "<<endl;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    cout << "Get a new goal!"<<endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planning");
    ros::NodeHandle nh("~");
    ros::Subscriber local_planner_sub =nh.subscribe<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 50, desired_vel_cb);
    
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/planning/goal", 10,goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    
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
        cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
        cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;


        rate.sleep();
    }

    return 0;

}
