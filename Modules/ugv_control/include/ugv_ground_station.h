#ifndef UGV_GROUND_STATION_H
#define UGV_GROUND_STATION_H

#include <Eigen/Eigen>
#include <math.h>
#include <boost/format.hpp>

#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/UgvCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "math_utils.h"
#include "printf_utils.h"

using namespace std;
#define NUM_POINT 2

//参数声明
int swarm_num;
const int max_swarm_num = 40; // indicate max num
string ugv_name[max_swarm_num+1];
int ugv_id[max_swarm_num+1];

ros::Subscriber command_sub[max_swarm_num+1];
ros::Subscriber ugv_state_sub[max_swarm_num+1];

prometheus_msgs::UgvState State_ugv[max_swarm_num+1];
prometheus_msgs::UgvCommand Command_ugv[max_swarm_num+1];
Eigen::Vector3f local_pos, global_pos, local_vel, global_vel, ugv_att;


void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos[0] = msg->pose.position.x;
    local_pos[1] = msg->pose.position.y;
    local_pos[2] = msg->pose.position.z;
}

void pos_cb2(const nav_msgs::Odometry::ConstPtr &msg)
{
    global_pos[0] = msg->pose.pose.position.x;
    global_pos[1] = msg->pose.pose.position.y;
    global_pos[2] = msg->pose.pose.position.z;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    local_vel[0] = msg->twist.linear.x;
    local_vel[1] = msg->twist.linear.y;
    local_vel[2] = msg->twist.linear.z;
}
void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);

    ugv_att[0] = euler_fcu[0];
    ugv_att[1] = euler_fcu[1];
    ugv_att[2] = euler_fcu[2];
}
void printf_test_state()
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

    // printf test
    cout << "local_pos   : " << local_pos[0] << " [ m ] "<< local_pos[1]<<" [ m ] "<< local_pos[2]<<" [ m ] "<<endl;
    cout << "local_vel     : " << local_vel[0] << " [m/s] "<< local_vel[1]<<" [m/s] "<< local_vel[2]<<" [m/s] "<<endl;
    cout << "global_pos : " << global_pos[0] << " [ m ] "<< global_pos[1]<<" [ m ] "<< global_pos[2]<<" [ m ] "<<endl;
    cout << "ugv_att        : " << ugv_att[0] * 180/M_PI <<" [deg] "<< ugv_att[1] * 180/M_PI << " [deg] "<< ugv_att[2] * 180/M_PI<<" [deg] "<<endl;

}

//函数声明
void ugv_command_cb_1(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[1] = *msg; }
void ugv_command_cb_2(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[2] = *msg; }
void ugv_command_cb_3(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[3] = *msg; }
void ugv_command_cb_4(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[4] = *msg; }
void ugv_command_cb_5(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[5] = *msg; }
void ugv_command_cb_6(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[6] = *msg; }
void ugv_command_cb_7(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[7] = *msg; }
void ugv_command_cb_8(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[8] = *msg; }
void ugv_command_cb_9(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[9] = *msg; }
void ugv_command_cb_10(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[10] = *msg; }
void ugv_command_cb_11(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[11] = *msg; }
void ugv_command_cb_12(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[12] = *msg; }
void ugv_command_cb_13(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[13] = *msg; }
void ugv_command_cb_14(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[14] = *msg; }
void ugv_command_cb_15(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[15] = *msg; }
void ugv_command_cb_16(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[16] = *msg; }
void ugv_command_cb_17(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[17] = *msg; }
void ugv_command_cb_18(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[18] = *msg; }
void ugv_command_cb_19(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[19] = *msg; }
void ugv_command_cb_20(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[20] = *msg; }
void ugv_command_cb_21(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[21] = *msg; }
void ugv_command_cb_22(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[22] = *msg; }
void ugv_command_cb_23(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[23] = *msg; }
void ugv_command_cb_24(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[24] = *msg; }
void ugv_command_cb_25(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[25] = *msg; }
void ugv_command_cb_26(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[26] = *msg; }
void ugv_command_cb_27(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[27] = *msg; }
void ugv_command_cb_28(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[28] = *msg; }
void ugv_command_cb_29(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[29] = *msg; }
void ugv_command_cb_30(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[30] = *msg; }
void ugv_command_cb_31(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[31] = *msg; }
void ugv_command_cb_32(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[32] = *msg; }
void ugv_command_cb_33(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[33] = *msg; }
void ugv_command_cb_34(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[34] = *msg; }
void ugv_command_cb_35(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[35] = *msg; }
void ugv_command_cb_36(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[36] = *msg; }
void ugv_command_cb_37(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[37] = *msg; }
void ugv_command_cb_38(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[38] = *msg; }
void ugv_command_cb_39(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[39] = *msg; }
void ugv_command_cb_40(const prometheus_msgs::UgvCommand::ConstPtr& msg) { Command_ugv[40] = *msg; }
void (*ugv_command_cb[max_swarm_num+1])(const prometheus_msgs::UgvCommand::ConstPtr&)={NULL,
    ugv_command_cb_1,ugv_command_cb_2,ugv_command_cb_3,ugv_command_cb_4,ugv_command_cb_5,
    ugv_command_cb_6,ugv_command_cb_7,ugv_command_cb_8,ugv_command_cb_9,ugv_command_cb_10,
    ugv_command_cb_11,ugv_command_cb_12,ugv_command_cb_13,ugv_command_cb_14,ugv_command_cb_15,
    ugv_command_cb_16,ugv_command_cb_17,ugv_command_cb_18,ugv_command_cb_19,ugv_command_cb_20,
    ugv_command_cb_21,ugv_command_cb_22,ugv_command_cb_23,ugv_command_cb_24,ugv_command_cb_25,
    ugv_command_cb_26,ugv_command_cb_27,ugv_command_cb_28,ugv_command_cb_29,ugv_command_cb_30,
    ugv_command_cb_31,ugv_command_cb_32,ugv_command_cb_33,ugv_command_cb_34,ugv_command_cb_35,
    ugv_command_cb_36,ugv_command_cb_37,ugv_command_cb_38,ugv_command_cb_39,ugv_command_cb_40};

void ugv_state_cb1(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[1] = *msg; }
void ugv_state_cb2(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[2] = *msg; }
void ugv_state_cb3(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[3] = *msg; }
void ugv_state_cb4(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[4] = *msg; }
void ugv_state_cb5(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[5] = *msg; }
void ugv_state_cb6(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[6] = *msg; }
void ugv_state_cb7(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[7] = *msg; }
void ugv_state_cb8(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[8] = *msg; }
void ugv_state_cb9(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[9] = *msg; }
void ugv_state_cb10(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[10] = *msg; }
void ugv_state_cb11(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[11] = *msg; }
void ugv_state_cb12(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[12] = *msg; }
void ugv_state_cb13(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[13] = *msg; }
void ugv_state_cb14(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[14] = *msg; }
void ugv_state_cb15(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[15] = *msg; }
void ugv_state_cb16(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[16] = *msg; }
void ugv_state_cb17(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[17] = *msg; }
void ugv_state_cb18(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[18] = *msg; }
void ugv_state_cb19(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[19] = *msg; }
void ugv_state_cb20(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[20] = *msg; }
void ugv_state_cb21(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[21] = *msg; }
void ugv_state_cb22(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[22] = *msg; }
void ugv_state_cb23(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[23] = *msg; }
void ugv_state_cb24(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[24] = *msg; }
void ugv_state_cb25(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[25] = *msg; }
void ugv_state_cb26(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[26] = *msg; }
void ugv_state_cb27(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[27] = *msg; }
void ugv_state_cb28(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[28] = *msg; }
void ugv_state_cb29(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[29] = *msg; }
void ugv_state_cb30(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[30] = *msg; }
void ugv_state_cb31(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[31] = *msg; }
void ugv_state_cb32(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[32] = *msg; }
void ugv_state_cb33(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[33] = *msg; }
void ugv_state_cb34(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[34] = *msg; }
void ugv_state_cb35(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[35] = *msg; }
void ugv_state_cb36(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[36] = *msg; }
void ugv_state_cb37(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[37] = *msg; }
void ugv_state_cb38(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[38] = *msg; }
void ugv_state_cb39(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[39] = *msg; }
void ugv_state_cb40(const prometheus_msgs::UgvState::ConstPtr& msg) { State_ugv[40] = *msg; }
void (*ugv_state_cb[max_swarm_num+1])(const prometheus_msgs::UgvState::ConstPtr&)={NULL,
    ugv_state_cb1,ugv_state_cb2,ugv_state_cb3,ugv_state_cb4,ugv_state_cb5,
    ugv_state_cb6,ugv_state_cb7,ugv_state_cb8,ugv_state_cb9,ugv_state_cb10,
    ugv_state_cb11,ugv_state_cb12,ugv_state_cb13,ugv_state_cb14,ugv_state_cb15,
    ugv_state_cb16,ugv_state_cb17,ugv_state_cb18,ugv_state_cb19,ugv_state_cb20,
    ugv_state_cb21,ugv_state_cb22,ugv_state_cb23,ugv_state_cb24,ugv_state_cb25,
    ugv_state_cb26,ugv_state_cb27,ugv_state_cb28,ugv_state_cb29,ugv_state_cb30,
    ugv_state_cb31,ugv_state_cb32,ugv_state_cb33,ugv_state_cb34,ugv_state_cb35,
    ugv_state_cb36,ugv_state_cb37,ugv_state_cb38,ugv_state_cb39,ugv_state_cb40};

void printf_ugv_state(int swarm_num, int ugv_id, string ugv_name, const prometheus_msgs::UgvState& Ugv_state, const prometheus_msgs::UgvCommand& UgvCommand)
{
    float x,y,z,yaw;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> "<< ugv_name << " State <<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "ugv_id: ["<< ugv_id <<"] ";
   //是否和飞控建立起连接
    if (Ugv_state.connected == true)
    {
        cout << "[ Connected ] ";
    }
    else
    {
        cout << "[ Unconnected ] ";
    }
    //是否上锁
    if (Ugv_state.armed == true)
    {
        cout << "[ Armed ] ";
    }
    else
    {
        cout << "[ DisArmed ] ";
    }

    if (Ugv_state.guided == true)
    {
        cout << "[ Guided ] ";
    }
    else
    {
        cout << "[ No-Guided ] ";
    }

    cout << "[ " << Ugv_state.mode<<" ] " <<endl;

    cout << "Position [X Y Z] : " << Ugv_state.position[0] << " [ m ] "<< Ugv_state.position[1]<<" [ m ] "<<Ugv_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << Ugv_state.velocity[0] << " [m/s] "<< Ugv_state.velocity[1]<<" [m/s] "<<Ugv_state.velocity[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << Ugv_state.attitude[0] * 180/M_PI <<" [deg] "<<Ugv_state.attitude[1] * 180/M_PI << " [deg] "<< Ugv_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;

    switch(UgvCommand.Mode)
    {
        case prometheus_msgs::UgvCommand::Start:
            if(UgvCommand.linear_vel[0] == 999)
            {
                cout << "Command: [ Start + Arming + Switching to OFFBOARD mode ] " <<endl;
            }else
            {
                cout << "Command: [ Start ] " <<endl;
            }
            break;

        case prometheus_msgs::UgvCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case prometheus_msgs::UgvCommand::Disarm:
            cout << "Command: [ Disarm ] " <<endl;
            break;

        case prometheus_msgs::UgvCommand::Point_Control:
            cout << "Command: [ Point_Control ] " <<endl;
            x = UgvCommand.position_ref[0];
            y = UgvCommand.position_ref[1];
            yaw = UgvCommand.yaw_ref;

            cout << "Pos_ref [X Y] : " << x  << " [ m ] "<< y <<" [ m ] "<<endl;
            cout << "Yaw           : " << yaw * 180/M_PI << " [deg] " <<endl;
            break;

        case prometheus_msgs::UgvCommand::Direct_Control:
            cout << "Command: [ Direct_Control ] " <<endl;
            x = UgvCommand.linear_vel[0];
            y = UgvCommand.linear_vel[1];
            z = UgvCommand.angular_vel;

            cout << "Vel [linear angular] : " << x  << " [m/s] "<< z <<" [rad/s] "<<endl;
            break;

        case prometheus_msgs::UgvCommand::ENU_Vel_Control:
            cout << "Command: [ ENU_Vel_Control ] " <<endl;
            x = UgvCommand.linear_vel[0];
            y = UgvCommand.linear_vel[1];
            z = UgvCommand.yaw_ref;

            cout << "Vel [velocity(x,y) yaw] : " << x  << " [m/s] "<< y  << " [m/s] "<< z <<" [ rad ] "<<endl;
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