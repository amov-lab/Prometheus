/***************************************************************************************************************************
* pos_controller_UDE.h
*
* Author: Qyp
*
* Update Time: 2019.4.12
*
* Introduction:  Position Controller using UDE method
*         1. Ref to Zhongqingchang's paper:
*     Uncertainty and Disturbance Estimator-Based Robust Trajectory Tracking Control for a Quadrotor in a Global Positioning System-Denied Environment
*         2. Ref to : https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/PositionControl.cpp
*         3. ThrottleToAttitude ref to https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/Utility/ControlMath.cpp
*         4. 没有考虑积分器清零的情况，在降落时 或者突然换方向机动时，积分器需要清0
*         5. 推力到欧拉角基本与PX4吻合，但是在极端情况下不吻合。如：z轴期望值为-100时。
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_UDE_H
#define POS_CONTROLLER_UDE_H

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <prometheus_control_utils.h>
#include <math_utils.h>


#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>


using namespace std;


class pos_controller_UDE
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_UDE(void):
            pos_UDE_nh("~")
        {
            pos_UDE_nh.param<float>("Quad/mass", Quad_MASS, 1.0);

            pos_UDE_nh.param<float>("Pos_ude/Kp_xy", Kp[0], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kp_xy", Kp[1], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kp_z", Kp[2], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_xy", Kd[0], 2.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_xy", Kd[1], 2.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_z", Kd[2], 2.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_xy", T_ude[0], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_xy", T_ude[1], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_z", T_ude[2], 1.0);

            pos_UDE_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 0.6);
            pos_UDE_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 0.6);
            pos_UDE_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 1.0);
            pos_UDE_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 0.3);
            pos_UDE_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 0.3);
            pos_UDE_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 1.0);
            pos_UDE_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 0.5);
            pos_UDE_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 0.5);
            pos_UDE_nh.param<float>("Limit/pz_int_max"   , int_max[2], 0.5);
            pos_UDE_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_UDE_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);  

            u_l      = Eigen::Vector3f(0.0,0.0,0.0);
            u_d      = Eigen::Vector3f(0.0,0.0,0.0);
            integral = Eigen::Vector3f(0.0,0.0,0.0);
        }

        //Quadrotor Parameter
        float Quad_MASS;


        //UDE control parameter
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f T_ude;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;

        float tilt_max;
        float int_start_error;

        //u_l for nominal contorol(PD), u_d for ude control(disturbance estimator)
        Eigen::Vector3f u_l,u_d;

        Eigen::Vector3f integral;

        prometheus_msgs::ControlOutput _ControlOutput;


        //Printf the UDE parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        prometheus_msgs::ControlOutput pos_controller(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State, float dt);

    private:

        ros::NodeHandle pos_UDE_nh;

};

prometheus_msgs::ControlOutput pos_controller_UDE::pos_controller(
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

    // UDE算法
    for (int i = 0; i < 3; i++)
    {
        u_l[i] = _Reference_State.acceleration_ref[i] + Kp[i] * pos_error[i] + Kd[i] * vel_error[i];
        u_d[i] = - 1.0 / T_ude[i] * (Kd[i] * pos_error[i] + vel_error[i] + Kp[i] * integral[i]);
    }

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) < int_start_error)
        {
            integral[i] += pos_error[i] * dt;
        }else
        {
            integral[i] = 0;
        }

        if(_DroneState.mode != "OFFBOARD")
        {
            integral[i] = 0;
        }

        if(abs(u_d[i]) > int_max[i])
        {
            cout << "u_d saturation! " << " [0-1-2] "<< i <<endl;
            cout << "[u_d]: "<< u_d[i]<<" [u_d_max]: "<<int_max[i]<<" [m/s] "<<endl;
        }

        u_d[i] = constrain_function(u_d[i], int_max[i]);
    }

    // 期望加速度
    accel_sp[0] = u_l[0] - u_d[0];
    accel_sp[1] = u_l[1] - u_d[1];
    accel_sp[2] = u_l[2] - u_d[2] + 9.8;

    // 期望推力 = 期望加速度 × 质量
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    thrust_sp =  prometheus_control_utils::accelToThrust(accel_sp, Quad_MASS, tilt_max);
    throttle_sp = prometheus_control_utils::thrustToThrottle(thrust_sp);

    for (int i=0; i<3; i++)
    {
        _ControlOutput.u_l[i] = u_l[i];
        _ControlOutput.u_d[i] = u_d[i];
        _ControlOutput.Thrust[i] = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }

    return _ControlOutput;
}

void pos_controller_UDE::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>  PD+UDE Position Controller  <<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);
    
    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;
    cout << "int [X Y Z] : " << integral[0] << " [N] "<< integral[1]<<" [N] "<<integral[2]<<" [N] "<<endl;
    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;
}

// 【打印参数函数】
void pos_controller_UDE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>UDE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout <<"Quad_MASS : "<< Quad_MASS << endl;


    cout <<"Kp_x : "<< Kp[0] << endl;
    cout <<"Kp_y : "<< Kp[1] << endl;
    cout <<"Kp_z : "<< Kp[2] << endl;

    cout <<"Kd_x : "<< Kd[0] << endl;
    cout <<"Kd_y : "<< Kd[1] << endl;
    cout <<"Kd_z : "<< Kd[2] << endl;

    cout <<"T_ude_x : "<< T_ude[0] << endl;
    cout <<"T_ude_y : "<< T_ude[1] << endl;
    cout <<"T_ude_z : "<< T_ude[2] << endl;

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
