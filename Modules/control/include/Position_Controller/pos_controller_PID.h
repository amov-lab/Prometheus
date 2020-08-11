/***************************************************************************************************************************
* pos_controller_PID.h
*
* Author: Qyp
*
* Update Time: 2019.6.29
*
* Introduction:  Position Controller using normal PID 
*                 output = a_ff + K_p * pos_error + K_d * vel_error + K_i * pos_error * dt;
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_PID_H
#define POS_CONTROLLER_PID_H

#include <math.h>
#include <command_to_mavros.h>
#include <prometheus_control_utils.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>


using namespace std;

class pos_controller_PID
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_PID(void):
            pos_pid_nh("~")
        {
            pos_pid_nh.param<float>("Quad/mass", Quad_MASS, 1.5);

            pos_pid_nh.param<float>("Pos_pid/Kp_xy", Kp[0], 2.5);
            pos_pid_nh.param<float>("Pos_pid/Kp_xy", Kp[1], 2.5);
            pos_pid_nh.param<float>("Pos_pid/Kp_z" , Kp[2], 2.5);
            pos_pid_nh.param<float>("Pos_pid/Kd_xy", Kd[0], 3.0);
            pos_pid_nh.param<float>("Pos_pid/Kd_xy", Kd[1], 3.0);
            pos_pid_nh.param<float>("Pos_pid/Kd_z" , Kd[2], 3.0);
            pos_pid_nh.param<float>("Pos_pid/Ki_xy", Ki[0], 0.5);
            pos_pid_nh.param<float>("Pos_pid/Ki_xy", Ki[1], 0.5);
            pos_pid_nh.param<float>("Pos_pid/Ki_z" , Ki[2], 0.5);
            
            pos_pid_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 10.0);
            pos_pid_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 10.0);
            pos_pid_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 10.0);
            pos_pid_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 10.0);
            pos_pid_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 10.0);
            pos_pid_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 10.0);
            pos_pid_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 10.0);
            pos_pid_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 10.0);
            pos_pid_nh.param<float>("Limit/pz_int_max"   , int_max[2], 10.0);
            pos_pid_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_pid_nh.param<float>("Limit/int_start_error"  , int_start_error, 10.0);

            integral = Eigen::Vector3f(0.0,0.0,0.0);
        }

        //Quadrotor Parameter
        float Quad_MASS;

        //PID parameter for the control law
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f Ki;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float tilt_max;
        float int_start_error;

        //积分项
        Eigen::Vector3f integral;

        //输出
        prometheus_msgs::ControlOutput _ControlOutput;


        //Printf the PID parameter
        void printf_param();

        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        prometheus_msgs::ControlOutput pos_controller(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State, float dt);

    private:
        ros::NodeHandle pos_pid_nh;

};

prometheus_msgs::ControlOutput pos_controller_PID::pos_controller(
    const prometheus_msgs::DroneState& _DroneState, 
    const prometheus_msgs::PositionReference& _Reference_State, float dt)
{
    Eigen::Vector3d accel_sp;
    
    // 计算误差项
    Eigen::Vector3f pos_error;
    Eigen::Vector3f vel_error;
    
    pos_error = prometheus_control_utils::cal_pos_error(_DroneState, _Reference_State);
    vel_error = prometheus_control_utils::cal_vel_error(_DroneState, _Reference_State);

    // 误差项限幅
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
        vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
    }

    // 期望加速度 = 加速度前馈 + PID
    for (int i=0; i<3; i++)
    {
        accel_sp[i] = _Reference_State.acceleration_ref[i] + Kp[i] * pos_error[i] + Kd[i] * vel_error[i] + Ki[i] * integral[i];
    }
    
    accel_sp[2] = accel_sp[2] + 9.8;

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) < int_start_error)
        {
            integral[i] += pos_error[i] * dt;

            if(abs(integral[i]) > int_max[i])
            {
                cout << "Integral saturation! " << " [0-1-2] "<< i <<endl;
                cout << "[integral]: "<< integral[i]<<" [int_max]: "<<int_max[i]<<" [m/s] "<<endl;
            }

            integral[i] = constrain_function(integral[i], int_max[i]);
        }else
        {
            integral[i] = 0;
        }

        // If not in OFFBOARD mode, set all intergral to zero.
        if(_DroneState.mode != "OFFBOARD")
        {
            integral[i] = 0;
        }
    }

    // 期望推力 = 期望加速度 × 质量
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    thrust_sp =  prometheus_control_utils::accelToThrust(accel_sp, Quad_MASS, tilt_max);
    throttle_sp = prometheus_control_utils::thrustToThrottle(thrust_sp);

    for (int i=0; i<3; i++)
    {
        _ControlOutput.Thrust[i] = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }

    return _ControlOutput;

}

void pos_controller_PID::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  PID Position Controller  <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);
}

// 【打印参数函数】
void pos_controller_PID::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;

    cout <<"Kp_x : "<< Kp[0] << endl;
    cout <<"Kp_y : "<< Kp[1] << endl;
    cout <<"Kp_z : "<< Kp[2] << endl;

    cout <<"Kd_x : "<< Kd[0] << endl;
    cout <<"Kd_y : "<< Kd[1] << endl;
    cout <<"Kd_z : "<< Kd[2] << endl;

    cout <<"Ki_x : "<< Ki[0] << endl;
    cout <<"Ki_y : "<< Ki[1] << endl;
    cout <<"Ki_z : "<< Ki[2] << endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"pxy_int_max : "<< int_max[0] << endl;
    cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;

}

#endif
