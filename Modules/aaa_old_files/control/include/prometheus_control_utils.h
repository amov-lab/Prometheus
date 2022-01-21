/***************************************************************************************************************************
* prometheus_control_utils.h
*
* Author: Qyp
*
* Update Time: 2019.7.6
***************************************************************************************************************************/
#ifndef PROMETHEUS_CONTORL_UTILS_H
#define PROMETHEUS_CONTORL_UTILS_H

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
#include <prometheus_msgs/LogMessageControl.h>
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


namespace prometheus_control_utils 
{


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

// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系，yaw_angle是当前偏航角[rad]
void rotation_yaw(float yaw_angle, float body_frame[2], float enu_frame[2])
{
    enu_frame[0] = body_frame[0] * cos(yaw_angle) - body_frame[1] * sin(yaw_angle);
    enu_frame[1] = body_frame[0] * sin(yaw_angle) + body_frame[1] * cos(yaw_angle);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 控 制 辅 助 函 数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
//计算位置误差
Eigen::Vector3f cal_pos_error(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State)
{
    Eigen::Vector3f pos_error;

    for (int i=0; i<3; i++)
    {
        pos_error[i] = _Reference_State.position_ref[i] - _DroneState.position[i];
    }

    return pos_error;
}

//计算速度误差
Eigen::Vector3f cal_vel_error(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State)
{
    Eigen::Vector3f vel_error;

    for (int i=0; i<3; i++)
    {
        vel_error[i] = _Reference_State.velocity_ref[i] - _DroneState.velocity[i];
    }

    return vel_error;
}

Eigen::Vector3d accelToThrust(const Eigen::Vector3d& accel_sp, float mass, float tilt_max)
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
    
    return thrust_sp;   
}

Eigen::Vector3d thrustToThrottle(const Eigen::Vector3d& thrust_sp)
{
    Eigen::Vector3d throttle_sp;

    //电机模型，可通过辨识得到，推力-油门曲线
    for (int i=0; i<3; i++)
    {
        throttle_sp[i] = MOTOR_P1 * pow(thrust_sp[i],4) + MOTOR_P2 * pow(thrust_sp[i],3) + MOTOR_P3 * pow(thrust_sp[i],2) + MOTOR_P4 * thrust_sp[i] + MOTOR_P5;
        // PX4内部默认假设 0.5油门为悬停推力 ， 在无人机重量为1kg时，直接除20得到0.5
        // throttle_sp[i] = thrust_sp[i]/20；
    }
    return throttle_sp; 
}

//Throttle to Attitude
//Thrust to Attitude
//Input: desired thrust (desired throttle [0,1]) and yaw_sp(rad)
//Output: desired attitude (quaternion)
prometheus_msgs::AttitudeReference ThrottleToAttitude(const Eigen::Vector3d& thr_sp, float yaw_sp)
{
    prometheus_msgs::AttitudeReference _AttitudeReference;
    Eigen::Vector3d att_sp;
    att_sp[2] = yaw_sp;

    // desired body_z axis = -normalize(thrust_vector)
    Eigen::Vector3d body_x, body_y, body_z;

    double thr_sp_length = thr_sp.norm();

    //cout << "thr_sp_length : "<< thr_sp_length << endl;

    if (thr_sp_length > 0.00001f) {
            body_z = thr_sp.normalized();

    } else {
            // no thrust, set Z axis to safe value
            body_z = Eigen::Vector3d(0.0f, 0.0f, 1.0f);
    }

    // vector of desired yaw direction in XY plane, rotated by PI/2
    Eigen::Vector3d y_C = Eigen::Vector3d(-sinf(yaw_sp),cosf(yaw_sp),0.0);

    if (fabsf(body_z(2)) > 0.000001f) {
            // desired body_x axis, orthogonal to body_z
            body_x = y_C.cross(body_z);

            // keep nose to front while inverted upside down
            if (body_z(2) < 0.0f) {
                    body_x = -body_x;
            }

            body_x.normalize();

    } else {
            // desired thrust is in XY plane, set X downside to construct correct matrix,
            // but yaw component will not be used actually
            body_x = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
            body_x(2) = 1.0f;
    }

    // desired body_y axis
    body_y = body_z.cross(body_x);

    Eigen::Matrix3d R_sp;

    // fill rotation matrix
    for (int i = 0; i < 3; i++) {
            R_sp(i, 0) = body_x(i);
            R_sp(i, 1) = body_y(i);
            R_sp(i, 2) = body_z(i);
    }

    Eigen::Quaterniond q_sp(R_sp);

    rotation_to_euler(R_sp, att_sp);

    //cout << "Desired euler [R P Y]: "<< att_sp[0]* 180/M_PI <<" [deg] " << att_sp[1]* 180/M_PI <<" [deg] "<< att_sp[2]* 180/M_PI <<" [deg] "<< endl;
    //cout << "Desired Thrust: "<< thr_sp_length<< endl;
//    cout << "q_sp [x y z w]: "<< q_sp.x() <<" [ ] " << q_sp.y() <<" [ ] "<<q_sp.z() <<" [ ] "<<q_sp.w() <<" [ ] "<<endl;
//    cout << "R_sp : "<< R_sp(0, 0) <<" " << R_sp(0, 1) <<" "<< R_sp(0, 2) << endl;
//    cout << "     : "<< R_sp(1, 0) <<" " << R_sp(1, 1) <<" "<< R_sp(1, 2) << endl;
//    cout << "     : "<< R_sp(2, 0) <<" " << R_sp(2, 1) <<" "<< R_sp(2, 2) << endl;


    _AttitudeReference.throttle_sp[0] = thr_sp[0];
    _AttitudeReference.throttle_sp[1] = thr_sp[1];
    _AttitudeReference.throttle_sp[2] = thr_sp[2];

    //期望油门
    _AttitudeReference.desired_throttle = thr_sp_length; 

    _AttitudeReference.desired_att_q.w = q_sp.w();
    _AttitudeReference.desired_att_q.x = q_sp.x();
    _AttitudeReference.desired_att_q.y = q_sp.y();
    _AttitudeReference.desired_att_q.z = q_sp.z();

    _AttitudeReference.desired_attitude[0] = att_sp[0];  
    _AttitudeReference.desired_attitude[1] = att_sp[1]; 
    _AttitudeReference.desired_attitude[2] = att_sp[2]; 

    return _AttitudeReference;
}

//random number Generation
//if a = 0 b =0, random_num = [-1,1]
//rand函数，C语言中用来产生一个随机数的函数
float random_num(float a, float b)
{
    float random_num;
    
    random_num = a * 2 * (((float)(rand() % 100))/100 - 0.5) + b;

    return random_num;
}

}
#endif
