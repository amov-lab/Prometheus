#ifndef CXY_GROUND_STATION_SUB_H
#define CXY_GROUND_STATION_SUB_H
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <math.h>

#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/UgvCommand.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include "formation_utils.h"
#include "printf_utils.h"


#define NUM_POINT 2
#define MAX_UAV_NUM 41
#define MAX_UGV_NUM 21

using namespace std;


prometheus_msgs::DroneState State_uav[MAX_UAV_NUM+1];
prometheus_msgs::SwarmCommand Command_uav[MAX_UAV_NUM+1];
nav_msgs::Odometry Odom_uav[MAX_UAV_NUM+1];

prometheus_msgs::UgvState State_ugv[MAX_UGV_NUM+1];
prometheus_msgs::UgvCommand Command_ugv[MAX_UGV_NUM+1];
nav_msgs::Odometry Odom_ugv[MAX_UGV_NUM+1];

void uav_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int id) 
{
    State_uav[id] = *msg;
}
void uav_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg, int id) 
{
    Command_uav[id] = *msg;
}

void ugv_state_cb(const prometheus_msgs::UgvState::ConstPtr& msg, int id) 
{
    State_ugv[id] = *msg;
}
void ugv_command_cb(const prometheus_msgs::UgvCommand::ConstPtr& msg, int id) 
{
    Command_ugv[id] = *msg;
}

void printf_uav_state(int swarm_num_uav, int uav_id, const prometheus_msgs::DroneState& _Drone_state, const prometheus_msgs::SwarmCommand& SwarmCommand)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    Eigen::MatrixXf formation;
    float x,y,z,yaw;
    cout <<">>>>>>>>>>>>>>>>>>>>>>UAV[" << uav_id <<  "] State" << "<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "UAV_id: ["<< uav_id <<"] ";
   //是否和飞控建立起连接
    if (_Drone_state.connected == true)
    {
        cout << "[ Connected ] ";
    }
    else
    {
        cout << "[ Unconnected ] ";
    }
    //是否上锁
    if (_Drone_state.armed == true)
    {
        cout << "[ Armed ] ";
    }
    else
    {
        cout << "[ DisArmed ] ";
    }

    cout << "[ " << _Drone_state.mode<<" ] " <<endl;

    cout << "Battery                : " << _Drone_state.battery_state << " [ V ] "<<endl;
    cout << "Position [X Y Z] : " << _Drone_state.position[0] << " [ m ] "<< _Drone_state.position[1]<<" [ m ] "<<_Drone_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << _Drone_state.velocity[0] << " [m/s] "<< _Drone_state.velocity[1]<<" [m/s] "<<_Drone_state.velocity[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << _Drone_state.attitude[0] * 180/M_PI <<" [deg] "<<_Drone_state.attitude[1] * 180/M_PI << " [deg] "<< _Drone_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;

    switch(SwarmCommand.Mode)
    {
        case prometheus_msgs::SwarmCommand::Idle:
            if(SwarmCommand.yaw_ref == 999)
            {
                cout << "Command: [ Idle + Arming + Switching to OFFBOARD mode ] " <<endl;
            }else
            {
                cout << "Command: [ Idle ] " <<endl;
            }
            break;

        case prometheus_msgs::SwarmCommand::Takeoff:
            cout << "Command: [ Takeoff ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Land:
            cout << "Command: [ Land ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Disarm:
            cout << "Command: [ Disarm ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Position_Control:
            if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::One_column)
            {
                cout << "Command: [ Position_Control ] [One_column] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
            {
                cout << "Command: [ Position_Control ] [Triangle] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Square)
            {
                cout << "Command: [ Position_Control ] [Square] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Circular)
            {
                cout << "Command: [ Position_Control ] [Circular] size: " << SwarmCommand.swarm_size <<endl;
            }
            
            formation = formation_utils::get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num_uav);

            x = SwarmCommand.position_ref[0] + formation(uav_id-1,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id-1,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id-1,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id-1,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Velocity_Control:
            cout << "Command: [ Velocity_Control ] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::Accel_Control:
            cout << "Command: [ Accel_Control ] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::Move:

            if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS)
            {
                cout << "Command: [ Move in XYZ_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS_BODY)
            {
                cout << "Command: [ Move in XYZ_POS_BODY] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            {
                cout << "Command: [ Move in XY_VEL_Z_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS_BODY)
            {
                cout << "Command: [ Move in XY_VEL_Z_POS_BODY] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::TRAJECTORY)
            {
                cout << "Command: [ Move in TRAJECTORY] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Velocity [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.velocity_ref[2]<<" [m/s] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else
            {
                cout << " wrong sub move mode. " <<endl;
            }
            break;
            
        case prometheus_msgs::SwarmCommand::User_Mode1:
            cout << "Command: [ User_Mode1 ] " <<endl;
            break;
    }
}

void printf_ugv_state(int swarm_num_uav, int ugv_id, const prometheus_msgs::UgvState& _ugv_state, const prometheus_msgs::UgvCommand& UgvCommand)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    float x,y,z,yaw;

    cout <<">>>>>>>>>>>>>>>>>>>>>>UGV[" << ugv_id <<  "] State" << "<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    // 状态打印
    cout << "Position [X Y Z] : " << _ugv_state.position[0] << " [ m ] "<< _ugv_state.position[1] <<" [ m ] "<< _ugv_state.position[2] <<" [ m ] "<<endl;
    // 合速度
    float vel_ugv = sqrtf((pow(_ugv_state.velocity[0],2) + pow(_ugv_state.velocity[1],2)));
    cout << "Velocity [X Y Z] : " << _ugv_state.velocity[0] << " [m/s] "<< _ugv_state.velocity[1] <<" [m/s] "<< _ugv_state.velocity[2] <<" [m/s] "<<endl;
    cout << "Velocity               : " << vel_ugv <<" [m/s] "<<endl;
    cout << "Yaw                       : " << _ugv_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;
    cout << "Battery                : " << _ugv_state.battery<<" [V] "<<endl;

    switch(UgvCommand.Mode)
    {
        case prometheus_msgs::UgvCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case prometheus_msgs::UgvCommand::Direct_Control_BODY:
            cout << "Command: [ Direct_Control ] " <<endl;
            x = UgvCommand.linear_vel[0];
            y = UgvCommand.linear_vel[1];
            z = UgvCommand.angular_vel;
            
            vel_ugv = sqrtf((pow(x,2) + pow(y,2)));
            cout << "Velocity [ linear(x y) ] : " << x  << " [m/s] " << y  << " [m/s] " <<endl;
            cout << "Velocity [ total ]            : " << vel_ugv  << " [m/s] " <<endl;
            cout << "Velocity [ angular ]     : " <<  z <<" [rad/s] "<<endl;
            break;

        case prometheus_msgs::UgvCommand::Point_Control:
            cout << "Command: [ Point_Control ] " <<endl;
            x = UgvCommand.position_ref[0];
            y = UgvCommand.position_ref[1];
            cout << "Pos_ref [X Y] : " << x  << " [ m ] "<< y <<" [ m ] "<<endl;
            break;

        case prometheus_msgs::UgvCommand::Path_Control:
            cout << "Command: [ Path_Control ] " <<endl;
            x = UgvCommand.position_ref[0];
            y = UgvCommand.position_ref[1];
            yaw = UgvCommand.yaw_ref;

            cout << "Pos_ref [X Y] : " << x  << " [ m ] "<< y <<" [ m ] "<<endl;
            cout << "Yaw           : "  << yaw * 180/M_PI << " [deg] " <<endl;
            break;
    }
}

#endif