#ifndef SWARM_GROUND_STATION_H
#define SWARM_GROUND_STATION_H

#include <Eigen/Eigen>
#include <math.h>

#include <prometheus_msgs/Message.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/LogMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "message_utils.h"
// #include "math_utils.h"
#include "formation_utils.h"

using namespace std;
#define NUM_POINT 2

//参数声明
int swarm_num;
const int max_swarm_num = 40; // indicate max num
string uav_name[max_swarm_num+1];
int uav_id[max_swarm_num+1];
bool flag_ros2groundstation;
prometheus_msgs::DroneState State_uav[max_swarm_num+1];
prometheus_msgs::SwarmCommand Command_uav[max_swarm_num+1];
geometry_msgs::PoseStamped ref_pose_uav[max_swarm_num+1];
ros::Subscriber command_sub[max_swarm_num+1];
ros::Subscriber drone_state_sub[max_swarm_num+1];
ros::Subscriber message_sub[max_swarm_num+1];
char *servInetAddr = "127.0.0.1"; //sever ip
string data;
char sendline[1024];
int socketfd;
struct sockaddr_in sockaddr;

//函数声明
void swarm_command_cb_1(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[1] = *msg; }
void swarm_command_cb_2(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[2] = *msg; }
void swarm_command_cb_3(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[3] = *msg; }
void swarm_command_cb_4(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[4] = *msg; }
void swarm_command_cb_5(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[5] = *msg; }
void swarm_command_cb_6(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[6] = *msg; }
void swarm_command_cb_7(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[7] = *msg; }
void swarm_command_cb_8(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[8] = *msg; }
void swarm_command_cb_9(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[9] = *msg; }
void swarm_command_cb_10(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[10] = *msg; }
void swarm_command_cb_11(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[11] = *msg; }
void swarm_command_cb_12(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[12] = *msg; }
void swarm_command_cb_13(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[13] = *msg; }
void swarm_command_cb_14(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[14] = *msg; }
void swarm_command_cb_15(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[15] = *msg; }
void swarm_command_cb_16(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[16] = *msg; }
void swarm_command_cb_17(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[17] = *msg; }
void swarm_command_cb_18(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[18] = *msg; }
void swarm_command_cb_19(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[19] = *msg; }
void swarm_command_cb_20(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[20] = *msg; }
void swarm_command_cb_21(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[21] = *msg; }
void swarm_command_cb_22(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[22] = *msg; }
void swarm_command_cb_23(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[23] = *msg; }
void swarm_command_cb_24(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[24] = *msg; }
void swarm_command_cb_25(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[25] = *msg; }
void swarm_command_cb_26(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[26] = *msg; }
void swarm_command_cb_27(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[27] = *msg; }
void swarm_command_cb_28(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[28] = *msg; }
void swarm_command_cb_29(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[29] = *msg; }
void swarm_command_cb_30(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[30] = *msg; }
void swarm_command_cb_31(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[31] = *msg; }
void swarm_command_cb_32(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[32] = *msg; }
void swarm_command_cb_33(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[33] = *msg; }
void swarm_command_cb_34(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[34] = *msg; }
void swarm_command_cb_35(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[35] = *msg; }
void swarm_command_cb_36(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[36] = *msg; }
void swarm_command_cb_37(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[37] = *msg; }
void swarm_command_cb_38(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[38] = *msg; }
void swarm_command_cb_39(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[39] = *msg; }
void swarm_command_cb_40(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[40] = *msg; }
void (*swarm_command_cb[max_swarm_num+1])(const prometheus_msgs::SwarmCommand::ConstPtr&)={NULL,
    swarm_command_cb_1,swarm_command_cb_2,swarm_command_cb_3,swarm_command_cb_4,swarm_command_cb_5,
    swarm_command_cb_6,swarm_command_cb_7,swarm_command_cb_8,swarm_command_cb_9,swarm_command_cb_10,
    swarm_command_cb_11,swarm_command_cb_12,swarm_command_cb_13,swarm_command_cb_14,swarm_command_cb_15,
    swarm_command_cb_16,swarm_command_cb_17,swarm_command_cb_18,swarm_command_cb_19,swarm_command_cb_20,
    swarm_command_cb_21,swarm_command_cb_22,swarm_command_cb_23,swarm_command_cb_24,swarm_command_cb_25,
    swarm_command_cb_26,swarm_command_cb_27,swarm_command_cb_28,swarm_command_cb_29,swarm_command_cb_30,
    swarm_command_cb_31,swarm_command_cb_32,swarm_command_cb_33,swarm_command_cb_34,swarm_command_cb_35,
    swarm_command_cb_36,swarm_command_cb_37,swarm_command_cb_38,swarm_command_cb_39,swarm_command_cb_40};

void drone_state_cb1(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[1] = *msg; }
void drone_state_cb2(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[2] = *msg; }
void drone_state_cb3(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[3] = *msg; }
void drone_state_cb4(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[4] = *msg; }
void drone_state_cb5(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[5] = *msg; }
void drone_state_cb6(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[6] = *msg; }
void drone_state_cb7(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[7] = *msg; }
void drone_state_cb8(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[8] = *msg; }
void drone_state_cb9(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[9] = *msg; }
void drone_state_cb10(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[10] = *msg; }
void drone_state_cb11(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[11] = *msg; }
void drone_state_cb12(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[12] = *msg; }
void drone_state_cb13(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[13] = *msg; }
void drone_state_cb14(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[14] = *msg; }
void drone_state_cb15(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[15] = *msg; }
void drone_state_cb16(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[16] = *msg; }
void drone_state_cb17(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[17] = *msg; }
void drone_state_cb18(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[18] = *msg; }
void drone_state_cb19(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[19] = *msg; }
void drone_state_cb20(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[20] = *msg; }
void drone_state_cb21(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[21] = *msg; }
void drone_state_cb22(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[22] = *msg; }
void drone_state_cb23(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[23] = *msg; }
void drone_state_cb24(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[24] = *msg; }
void drone_state_cb25(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[25] = *msg; }
void drone_state_cb26(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[26] = *msg; }
void drone_state_cb27(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[27] = *msg; }
void drone_state_cb28(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[28] = *msg; }
void drone_state_cb29(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[29] = *msg; }
void drone_state_cb30(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[30] = *msg; }
void drone_state_cb31(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[31] = *msg; }
void drone_state_cb32(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[32] = *msg; }
void drone_state_cb33(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[33] = *msg; }
void drone_state_cb34(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[34] = *msg; }
void drone_state_cb35(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[35] = *msg; }
void drone_state_cb36(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[36] = *msg; }
void drone_state_cb37(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[37] = *msg; }
void drone_state_cb38(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[38] = *msg; }
void drone_state_cb39(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[39] = *msg; }
void drone_state_cb40(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[40] = *msg; }
void (*drone_state_cb[max_swarm_num+1])(const prometheus_msgs::DroneState::ConstPtr&)={NULL,
    drone_state_cb1,drone_state_cb2,drone_state_cb3,drone_state_cb4,drone_state_cb5,
    drone_state_cb6,drone_state_cb7,drone_state_cb8,drone_state_cb9,drone_state_cb10,
    drone_state_cb11,drone_state_cb12,drone_state_cb13,drone_state_cb14,drone_state_cb15,
    drone_state_cb16,drone_state_cb17,drone_state_cb18,drone_state_cb19,drone_state_cb20,
    drone_state_cb21,drone_state_cb22,drone_state_cb23,drone_state_cb24,drone_state_cb25,
    drone_state_cb26,drone_state_cb27,drone_state_cb28,drone_state_cb29,drone_state_cb30,
    drone_state_cb31,drone_state_cb32,drone_state_cb33,drone_state_cb34,drone_state_cb35,
    drone_state_cb36,drone_state_cb37,drone_state_cb38,drone_state_cb39,drone_state_cb40};

void printf_swarm_state(int swarm_num, int uav_id, string uav_name, const prometheus_msgs::DroneState& _Drone_state, const prometheus_msgs::SwarmCommand& SwarmCommand)
{
    Eigen::MatrixXf formation;
    float x,y,z,yaw;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> "<< uav_name << " State <<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "uav_id: ["<< uav_id <<"] ";
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
    //是否在地面
    if (_Drone_state.landed == true)
    {
        cout << "[ Ground ] ";
    }
    else
    {
        cout << "[ Air ] ";
    }

    cout << "[ " << _Drone_state.mode<<" ] " <<endl;

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
            
            formation = formation_utils::get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);

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
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            {
                cout << "Command: [ Move in XY_VEL_Z_POS] " <<endl;
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

#endif