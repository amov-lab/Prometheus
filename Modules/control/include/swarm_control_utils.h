/***************************************************************************************************************************
* swarm_control_utils.h
*
* Author: Qyp
*
* Update Time: 2019.7.6
***************************************************************************************************************************/
#ifndef SWARM_CONTORL_UTILS_H
#define SWARM_CONTORL_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <command_to_mavros.h>
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

using namespace std;

#define NUM_POINT 2
#define NUM_MOTOR 4

#define MOTOR_P1 -0.00069
#define MOTOR_P2 0.01271
#define MOTOR_P3 -0.07948
#define MOTOR_P4 0.3052
#define MOTOR_P5 0.008775

#define thrust_max_single_motor 6.0

Eigen::MatrixXf formation;

namespace swarm_control_utils 
{
//输入参数：　阵型，阵型基本尺寸，集群数量
Eigen::MatrixXf get_formation_separation(int swarm_shape, float swarm_size, int swarm_num)
{
    //矩阵大小为　swarm_num＊4 , 对应　x,y,z,yaw四个自由度的分离量
    Eigen::MatrixXf seperation(swarm_num,4); 

    // one_column shape
    // 横向一字型，虚拟领机位置为中心位置，其余飞机根据数量向左右增加
    if(swarm_shape == prometheus_msgs::SwarmCommand::One_column)
    {
        if(swarm_num == 3)
        {
            //　数量为3时，１号机即虚拟领机位置
            seperation(0,0) = 0.0;
            seperation(0,1) = 0.0;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = 0.0;
            seperation(1,1) = swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 0.0;
            seperation(2,1) = - 1 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;
        }

        if(swarm_num == 5)
        {
            //　数量为５时，１号机即虚拟领机位置
            seperation(0,0) = 0.0;
            seperation(0,1) = 0.0;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = 0.0;
            seperation(1,1) = swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 0.0;
            seperation(2,1) = - 1 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = 0.0;
            seperation(3,1) = 2 * swarm_size;  
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = 0.0;
            seperation(4,1) = - 2 * swarm_size;  
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;
        }
    }

    // one_row shape
    // 竖向一字型，虚拟领机位置为中心位置，其余飞机根据数量向左右增加
    if(swarm_shape == prometheus_msgs::SwarmCommand::One_row)
    {
        if(swarm_num == 3)
        {
            //　数量为3时，１号机即虚拟领机位置
            seperation(0,0) = swarm_size;
            seperation(0,1) = 0.0;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = 0.0;
            seperation(1,1) = 0.0;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = - 1 * swarm_size;
            seperation(2,1) = 0.0;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;
        }

        if(swarm_num == 5)
        {
            //　数量为５时，１号机即虚拟领机位置
            seperation(0,0) = 2 * swarm_size;
            seperation(0,1) = 0.0;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = swarm_size;
            seperation(1,1) = 0.0;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 0.0;
            seperation(2,1) = 0.0; 
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = -1 * swarm_size;  
            seperation(3,1) = 0.0;
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = - 2 * swarm_size; 
            seperation(4,1) = 0.0;
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;
        }
    }

    // triangle shape
    // 三角型，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
    {
        if(swarm_num == 3)
        {
            //　数量为５时，
            seperation(0,0) = swarm_size;
            seperation(0,1) = 0.0;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = 0.0;
            seperation(1,1) = 0.5 * swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 0.0;
            seperation(2,1) = - 0.5 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;
        }

        if(swarm_num == 5)
        {
            //　数量为５时，
            seperation(0,0) = swarm_size;
            seperation(0,1) = 0.0;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = 0.0;
            seperation(1,1) = 0.5 * swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 0.0;
            seperation(2,1) = - 0.5 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = -1 * swarm_size;
            seperation(3,1) = 1 * swarm_size;  
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = -1 * swarm_size;
            seperation(4,1) = -1 * swarm_size;  
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;
        }
    }

    return seperation;
}

void printf_swarm_state(int swarm_num, int uav_id, string uav_name, const prometheus_msgs::DroneState& _Drone_state, const prometheus_msgs::SwarmCommand& SwarmCommand)
{
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
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::One_row)
            {
                cout << "Command: [ Position_Control ] [One_row] size: " << SwarmCommand.swarm_size <<endl;
            }
            
            formation = get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);

            x = SwarmCommand.position_ref[0] + formation(uav_id-1,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id-1,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id-1,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id-1,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::Velocity_Control:
            cout << "Command: [ Velocity_Control ] " <<endl;
            formation = get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);

            x = SwarmCommand.position_ref[0] + formation(uav_id-1,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id-1,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id-1,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id-1,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::Accel_Control:
            cout << "Command: [ Accel_Control ] " <<endl;
            formation = get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);
            
            x = SwarmCommand.position_ref[0] + formation(uav_id-1,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id-1,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id-1,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id-1,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::User_Mode1:
            cout << "Command: [ User_Mode1 ] " <<endl;
            break;

    }

}

Eigen::Vector3d accelToThrottle(const Eigen::Vector3d& accel_sp, float mass, float tilt_max)
{
    Eigen::Vector3d thrust_sp;

    //除以电机个数得到单个电机的期望推力
    thrust_sp = mass * accel_sp / NUM_MOTOR;

    // 推力限幅，根据最大倾斜角及最大油门
    float thrust_max_XY_tilt = fabs(thrust_sp[2]) * tanf(tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(thrust_max_single_motor * thrust_max_single_motor - pow(thrust_sp[2],2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    if ((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)) > pow(thrust_max_XY,2)) 
    {
        float mag = sqrtf((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)));
        thrust_sp[0] = thrust_sp[0] / mag * thrust_max_XY;
        thrust_sp[1] = thrust_sp[1] / mag * thrust_max_XY;
    }

    Eigen::Vector3d throttle_sp;

    //电机模型，可通过辨识得到，推力-油门曲线
    for (int i=0; i<3; i++)
    {
        throttle_sp[i] = MOTOR_P1 * pow(thrust_sp[i],4) + MOTOR_P2 * pow(thrust_sp[i],3) + MOTOR_P3 * pow(thrust_sp[i],2) + MOTOR_P4 * thrust_sp[i] + MOTOR_P5;
        // PX4内部默认假设 0.5油门为悬停推力 ， 在无人机重量为1kg时，直接除20得到0.5
        // throttle_sp[i] = thrust_sp[i]/20;
    }

    return throttle_sp;   
}



}
#endif
