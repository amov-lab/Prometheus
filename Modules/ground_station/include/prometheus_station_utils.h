/***************************************************************************************************************************
* prometheus_station_utils.h
*
* Author: Qyp
*
* Update Time: 2019.7.6
***************************************************************************************************************************/
#ifndef PROMETHEUS_STATION_UTILS_H
#define PROMETHEUS_STATION_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <prometheus_msgs/Message.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/LogMessageControl.h>
#include <prometheus_msgs/LogMessageDetection.h>
#include <prometheus_msgs/LogMessagePlanning.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace std;

#define NUM_POINT 2




namespace prometheus_station_utils 
{

// 五种状态机
enum MISSION_TYPE
{
    CONTROL,
    LAND,
    PLAN,
    TRACK,
};

// 打印上层控制指令  
void printf_command_control(const prometheus_msgs::ControlCommand& _ControlCommand)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Control Command <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "Source: [ "<< _ControlCommand.source << " ]  Command_ID: "<< _ControlCommand.Command_ID <<endl;

    switch(_ControlCommand.Mode)
    {
        case prometheus_msgs::ControlCommand::Idle:
            
            if(_ControlCommand.Reference_State.yaw_ref == 999)
            {
                cout << "Command: [ Idle + Arming + Switching to OFFBOARD mode ] " <<endl;
            }else
            {
                cout << "Command: [ Idle ] " <<endl;
            }
            
            break;

        case prometheus_msgs::ControlCommand::Takeoff:
            cout << "Command: [ Takeoff ] " <<endl;
            break;

        case prometheus_msgs::ControlCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case prometheus_msgs::ControlCommand::Land:
            cout << "Command: [ Land ] " <<endl;
            break;

        case prometheus_msgs::ControlCommand::Move:
            if(_ControlCommand.Reference_State.Move_mode == prometheus_msgs::PositionReference::XYZ_POS)
            {
                cout << "Command: [ Move ] " << "Move_mode: [ XYZ_POS ] " <<endl;
            }else if(_ControlCommand.Reference_State.Move_mode == prometheus_msgs::PositionReference::XY_POS_Z_VEL)
            {
                cout << "Command: [ Move ] " << "Move_mode: [ XY_POS_Z_VEL ] " <<endl;
            }else if(_ControlCommand.Reference_State.Move_mode == prometheus_msgs::PositionReference::XY_VEL_Z_POS)
            {
                cout << "Command: [ Move ] " << "Move_mode: [ XY_VEL_Z_POS ] " <<endl;
            }else if(_ControlCommand.Reference_State.Move_mode == prometheus_msgs::PositionReference::XYZ_VEL)
            {
                cout << "Command: [ Move ] " << "Move_mode: [ XYZ_VEL ] " <<endl;
            }else if(_ControlCommand.Reference_State.Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
            {
                cout << "Command: [ Move ] " << "Move_mode: [ TRAJECTORY ] " <<endl;
            }

            if(_ControlCommand.Reference_State.Move_frame == prometheus_msgs::PositionReference::ENU_FRAME)
            {
                cout << "Move_frame: [ ENU_FRAME ] " <<endl;
            }else if(_ControlCommand.Reference_State.Move_frame == prometheus_msgs::PositionReference::BODY_FRAME)
            {
                cout << "Move_frame: [ BODY_FRAME ] " <<endl;
            }

            cout << "Position [X Y Z] : " << _ControlCommand.Reference_State.position_ref[0] << " [ m ] "<< _ControlCommand.Reference_State.position_ref[1]<<" [ m ] "<< _ControlCommand.Reference_State.position_ref[2]<<" [ m ] "<<endl;
            cout << "Velocity [X Y Z] : " << _ControlCommand.Reference_State.velocity_ref[0] << " [m/s] "<< _ControlCommand.Reference_State.velocity_ref[1]<<" [m/s] "<< _ControlCommand.Reference_State.velocity_ref[2]<<" [m/s] "<<endl;
            cout << "Acceleration [X Y Z] : " << _ControlCommand.Reference_State.acceleration_ref[0] << " [m/s^2] "<< _ControlCommand.Reference_State.acceleration_ref[1]<<" [m/s^2] "<< _ControlCommand.Reference_State.acceleration_ref[2]<<" [m/s^2] "<<endl;

            cout << "Yaw : "  << _ControlCommand.Reference_State.yaw_ref* 180/M_PI << " [deg] " <<endl;

            break;

        case prometheus_msgs::ControlCommand::Disarm:
            cout << "Command: [ Disarm ] " <<endl;
            break;

        case prometheus_msgs::ControlCommand::User_Mode1:
            cout << "Command: [ User_Mode1 ] " <<endl;
            break;
        
        case prometheus_msgs::ControlCommand::User_Mode2:
            cout << "Command: [ User_Mode2 ] " <<endl;
            break;
    }

}



// 打印无人机状态
void prinft_drone_state(const prometheus_msgs::DroneState& _Drone_state)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>   Drone State   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "Time: " << _Drone_state.time_from_start <<" [s] ";

    //是否和飞控建立起连接
    if (_Drone_state.connected == true)
    {
        cout << " [ Connected ]";
    }
    else
    {
        cout << " [ Unconnected ]";
    }

    //是否上锁
    if (_Drone_state.armed == true)
    {
        cout << " [ Armed ]";
    }
    else
    {
        cout << " [ DisArmed ]";
    }

    //是否在地面
    if (_Drone_state.landed == true)
    {
        cout << " [ Ground ] ";
    }
    else
    {
        cout << " [ Air ] ";
    }

    cout << "[ " << _Drone_state.mode<<" ] " <<endl;

    cout << "Position [X Y Z] : " << _Drone_state.position[0] << " [ m ] "<< _Drone_state.position[1]<<" [ m ] "<<_Drone_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << _Drone_state.velocity[0] << " [m/s] "<< _Drone_state.velocity[1]<<" [m/s] "<<_Drone_state.velocity[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << _Drone_state.attitude[0] * 180/M_PI <<" [deg] "<<_Drone_state.attitude[1] * 180/M_PI << " [deg] "<< _Drone_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;
    cout << "Att_rate [R P Y] : " << _Drone_state.attitude_rate[0] * 180/M_PI <<" [deg/s] "<<_Drone_state.attitude_rate[1] * 180/M_PI << " [deg/s] "<< _Drone_state.attitude_rate[2] * 180/M_PI<<" [deg/s] "<<endl;
}

// 打印位置控制器输出结果
void prinft_attitude_reference(const prometheus_msgs::AttitudeReference& _AttitudeReference)
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

    cout << "Attitude_sp [R P Y]  : " << _AttitudeReference.desired_attitude[0] * 180/M_PI <<" [deg]  "<<_AttitudeReference.desired_attitude[1] * 180/M_PI << " [deg]  "<< _AttitudeReference.desired_attitude[2] * 180/M_PI<<" [deg] "<<endl;
    cout << "Throttle_sp [ 0-1 ]  : " << _AttitudeReference.desired_throttle <<endl;
}

void prinft_ref_pose(const geometry_msgs::PoseStamped& ref_pose)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>> Ref Pose <<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
      
    cout << "Ref_position [X Y Z] : " << ref_pose.pose.position.x <<" [m] "<< ref_pose.pose.position.y <<" [m] " << ref_pose.pose.position.z <<" [m] "<<endl;
}


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 其 他 函 数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}


}
#endif
