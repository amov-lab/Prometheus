/***************************************************************************************************************************
* pos_controller_NE.h
*
* Author: Qyp
*
* Update Time: 2021.3.5
*
* Introduction:  Position Controller using NE+UDE method
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_NE_H
#define POS_CONTROLLER_NE_H

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <prometheus_control_utils.h>
#include <math_utils.h>

#include <Filter/LowPassFilter.h>
#include <Filter/HighPassFilter.h>
#include <Filter/LeadLagFilter.h>

#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>


using namespace std;
 
class pos_controller_NE
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_NE(void):
            pos_NE_nh("~")
        {
            pos_NE_nh.param<float>("Quad/mass", Quad_MASS, 1.0);

            pos_NE_nh.param<float>("Pos_ne/Kp_xy", Kp[0], 1.0);
            pos_NE_nh.param<float>("Pos_ne/Kp_xy", Kp[1], 1.0);
            pos_NE_nh.param<float>("Pos_ne/Kp_z",  Kp[2], 1.0);
            pos_NE_nh.param<float>("Pos_ne/Kd_xy", Kd[0], 2.0);
            pos_NE_nh.param<float>("Pos_ne/Kd_xy", Kd[1], 2.0);
            pos_NE_nh.param<float>("Pos_ne/Kd_z",  Kd[2], 2.0);
            pos_NE_nh.param<float>("Pos_ne/T_ude_xy", T_ude[0], 1.0);
            pos_NE_nh.param<float>("Pos_ne/T_ude_xy", T_ude[1], 1.0);
            pos_NE_nh.param<float>("Pos_ne/T_ude_z", T_ude[2], 1.0);
            pos_NE_nh.param<float>("Pos_ne/T_ne", T_ne, 1.0);

            pos_NE_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 0.6);
            pos_NE_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 0.6);
            pos_NE_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 1.0);
            pos_NE_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 0.3);
            pos_NE_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 0.3);
            pos_NE_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 1.0);
            pos_NE_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 0.5);
            pos_NE_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 0.5);
            pos_NE_nh.param<float>("Limit/pz_int_max"   , int_max[2], 0.5);  
            pos_NE_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_NE_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            u_l             = Eigen::Vector3f(0.0,0.0,0.0);
            u_d             = Eigen::Vector3f(0.0,0.0,0.0);
            integral        = Eigen::Vector3f(0.0,0.0,0.0);
            integral_LLF    = Eigen::Vector3f(0.0,0.0,0.0);
            NoiseEstimator  = Eigen::Vector3f(0.0,0.0,0.0);
            output_LLF      = Eigen::Vector3f(0.0,0.0,0.0);
            pos_initial     = Eigen::Vector3d(0.0,0.0,0.0);
            set_filter();
        }


        //Quadrotor Parameter
        float Quad_MASS;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;

        float tilt_max;
        float int_start_error;

        //NE control parameter
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f T_ude;
        float T_ne;
        prometheus_msgs::ControlOutput _ControlOutput;


        //Filter for NE
        LowPassFilter LPF_x;
        LowPassFilter LPF_y;
        LowPassFilter LPF_z;

        HighPassFilter HPF_x;
        HighPassFilter HPF_y;
        HighPassFilter HPF_z;

        LeadLagFilter LLF_x;
        LeadLagFilter LLF_y;
        LeadLagFilter LLF_z;

        //u_l for nominal contorol(PD), u_d for NE control(disturbance estimator)
        Eigen::Vector3f u_l,u_d;

        Eigen::Vector3f integral;

        Eigen::Vector3f integral_LLF;

        Eigen::Vector3f output_LLF;

        Eigen::Vector3d pos_initial;

        Eigen::Vector3f NoiseEstimator;


        //Printf the NE parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        prometheus_msgs::ControlOutput pos_controller(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State, float dt);

        void set_initial_pos(const Eigen::Vector3d& pos);

        void set_filter();

    private:

        ros::NodeHandle pos_NE_nh;

};

void pos_controller_NE::set_filter()
{
    LPF_x.set_Time_constant(T_ne);
    LPF_y.set_Time_constant(T_ne);
    LPF_z.set_Time_constant(T_ne);

    HPF_x.set_Time_constant(T_ne);
    HPF_y.set_Time_constant(T_ne);
    HPF_z.set_Time_constant(T_ne);

    LLF_x.set_Time_constant(T_ne, Kd[0]);
    LLF_y.set_Time_constant(T_ne, Kd[1]);
    LLF_z.set_Time_constant(T_ne, Kd[2]);
}

void pos_controller_NE::set_initial_pos(const Eigen::Vector3d& pos)
{
    pos_initial = pos;
}

prometheus_msgs::ControlOutput pos_controller_NE::pos_controller(
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

    //Noise estimator
    NoiseEstimator[0] = LPF_x.apply(_DroneState.velocity[0], dt) + HPF_x.apply(pos_initial[0] - _DroneState.position[0], dt);
    NoiseEstimator[1] = LPF_y.apply(_DroneState.velocity[1], dt) + HPF_y.apply(pos_initial[1] - _DroneState.position[1], dt);
    NoiseEstimator[2] = LPF_z.apply(_DroneState.velocity[2], dt) + HPF_z.apply(pos_initial[2] - _DroneState.position[2], dt);

    //u_l
    for (int i = 0; i < 3; i++)
    {
       u_l[i] = _Reference_State.acceleration_ref[i] +  Kp[i] * pos_error[i] + Kd[i] * ( vel_error[i] + NoiseEstimator[i]);
    }

    //UDE term
    Eigen::Vector3f input_LLF;

    for (int i = 0; i < 3; i++)
    {
        integral_LLF[i] = integral_LLF[i] +  _DroneState.velocity[i] * dt;
        input_LLF[i] =  integral_LLF[i] - _DroneState.position[i] + pos_initial[i];
    }

    output_LLF[0] = LLF_x.apply(input_LLF[0], dt);
    output_LLF[1] = LLF_y.apply(input_LLF[1], dt);
    output_LLF[2] = LLF_z.apply(input_LLF[2], dt);

    for (int i = 0; i < 3; i++)
    {
        u_d[i] = 1.0 / T_ude[i] * ( _DroneState.velocity[i] - output_LLF[i] - integral[i] );
    }

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        integral[i] = integral[i] +  ( _Reference_State.acceleration_ref[i] +  Kp[i] * pos_error[i] + Kd[i] * vel_error[i]) * dt;

        // If not in OFFBOARD mode, set all intergral to zero.
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
        _ControlOutput.NE[i] = NoiseEstimator[i];
    }

    return _ControlOutput;
}

void pos_controller_NE::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  NE Position Controller  <<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << "NoiseEstimator [X Y Z] : " <<  Quad_MASS *Kd[0] * NoiseEstimator[0] << " [N] "<<Quad_MASS *Kd[1] * NoiseEstimator[1]<<" [N] "<<Quad_MASS *Kd[2] *NoiseEstimator[2]<<" [N] "<<endl;

    cout << "output_LLF [X Y Z] : " << output_LLF[0] << " [N] "<< output_LLF[1]<<" [N] "<<output_LLF[2]<<" [N] "<<endl;

    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;

    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;
}

// 【打印参数函数】
void pos_controller_NE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>NE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;

    cout <<"Kp_X : "<< Kp[0] << endl;
    cout <<"Kp_Y : "<< Kp[1] << endl;
    cout <<"Kp_Z : "<< Kp[2] << endl;

    cout <<"Kd_X : "<< Kd[0] << endl;
    cout <<"Kd_Y : "<< Kd[1] << endl;
    cout <<"Kd_Z : "<< Kd[2] << endl;

    cout <<"NE_T_X : "<< T_ude[0] << endl;
    cout <<"NE_T_Y : "<< T_ude[1] << endl;
    cout <<"NE_T_Z : "<< T_ude[2] << endl;
    cout <<"T_ne : "<< T_ne << endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"pxy_int_max : "<< int_max[0] << endl;
    cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;

    cout <<"Filter_LPFx : "<< LPF_x.get_Time_constant()<<" Filter_LPFy : "<< LPF_y.get_Time_constant()<<" Filter_LPFz : "<< LPF_z.get_Time_constant() << endl;
    cout <<"Filter_HPFx : "<< HPF_x.get_Time_constant()<<" Filter_HPFy : "<< HPF_y.get_Time_constant()<<" Filter_HPFz : "<< HPF_z.get_Time_constant() << endl;
    cout <<"Filter_LLFx : "<< LLF_x.get_Time_constant()<<" Filter_LLFy : "<< LLF_y.get_Time_constant()<<" Filter_LLFz : "<< LLF_z.get_Time_constant() << endl;

    cout <<"kd_LLFx : "<< LLF_x.get_Kd() <<" kd_LLFy : "<< LLF_y.get_Kd() <<" kd_LLFz : "<< LLF_z.get_Kd() << endl;


}


#endif