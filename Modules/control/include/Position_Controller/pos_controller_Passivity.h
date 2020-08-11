/***************************************************************************************************************************
* pos_controller_passivity.h
*
* Author: Qyp
*
* Update Time: 2019.5.1
*
* Introduction:  Position Controller using passivity+UDE method
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_PASSIVITY_H
#define POS_CONTROLLER_PASSIVITY_H

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <prometheus_control_utils.h>
#include <math_utils.h>
#include <Filter/LowPassFilter.h>
#include <Filter/HighPassFilter.h>

#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>


using namespace std;
 
class pos_controller_passivity
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_passivity(void):
            pos_passivity_nh("~")
        {
            pos_passivity_nh.param<float>("Quad/mass", Quad_MASS, 1.0);


            pos_passivity_nh.param<float>("Pos_passivity/Kp_xy", Kp[0], 1.0);
            pos_passivity_nh.param<float>("Pos_passivity/Kp_xy", Kp[1], 1.0);
            pos_passivity_nh.param<float>("Pos_passivity/Kp_z",  Kp[2], 1.0);
            pos_passivity_nh.param<float>("Pos_passivity/Kd_xy", Kd[0], 2.0);
            pos_passivity_nh.param<float>("Pos_passivity/Kd_xy", Kd[1], 2.0);
            pos_passivity_nh.param<float>("Pos_passivity/Kd_z",  Kd[2], 2.0);
            pos_passivity_nh.param<float>("Pos_passivity/T_ude_xy", T_ude[0], 1.0);
            pos_passivity_nh.param<float>("Pos_passivity/T_ude_xy", T_ude[1], 1.0);
            pos_passivity_nh.param<float>("Pos_passivity/T_ude_z",  T_ude[2], 1.0);
            pos_passivity_nh.param<float>("Pos_passivity/T_ps", T_ps, 1.0);

            pos_passivity_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 0.6);
            pos_passivity_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 0.6);
            pos_passivity_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 1.0);
            pos_passivity_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 0.3);
            pos_passivity_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 0.3);
            pos_passivity_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 1.0);
            pos_passivity_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 0.5);
            pos_passivity_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 0.5);
            pos_passivity_nh.param<float>("Limit/pz_int_max"   , int_max[2], 0.5);  
            pos_passivity_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_passivity_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            u_l        = Eigen::Vector3f(0.0,0.0,0.0);
            u_d        = Eigen::Vector3f(0.0,0.0,0.0);
            integral   = Eigen::Vector3f(0.0,0.0,0.0);

            y1_k       = Eigen::Vector3f(0.0,0.0,0.0);
            y2_k       = Eigen::Vector3f(0.0,0.0,0.0);
            y3_k       = Eigen::Vector3f(0.0,0.0,0.0);
            z_k        = Eigen::Vector3f(0.0,0.0,0.0);

            set_filter();

        }

        //Quadrotor Parameter
        float Quad_MASS;

        //passivity control parameter
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f T_ude;
        float T_ps;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float tilt_max;
        float int_start_error;

        //u_l for nominal contorol(PD), u_d for passivity control(disturbance estimator)
        Eigen::Vector3f u_l,u_d;
        Eigen::Vector3f integral;
        prometheus_msgs::ControlOutput _ControlOutput;


        HighPassFilter HPF_pos_error_x;
        HighPassFilter HPF_pos_error_y;
        HighPassFilter HPF_pos_error_z;

        HighPassFilter HPF_pos_x;
        HighPassFilter HPF_pos_y;
        HighPassFilter HPF_pos_z;

        LowPassFilter LPF_pos_error_x;
        LowPassFilter LPF_pos_error_y;
        LowPassFilter LPF_pos_error_z;

        LowPassFilter LPF_int_x;
        LowPassFilter LPF_int_y;
        LowPassFilter LPF_int_z;

        Eigen::Vector3f z_k;

        Eigen::Vector3f y1_k,y2_k,y3_k;


        //Printf the passivity parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        void set_filter();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        prometheus_msgs::ControlOutput pos_controller(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State, float dt);

    private:

        ros::NodeHandle pos_passivity_nh;

};
void pos_controller_passivity::set_filter()
{   
    HPF_pos_error_x.set_Time_constant(T_ps);
    HPF_pos_error_y.set_Time_constant(T_ps);
    HPF_pos_error_z.set_Time_constant(T_ps);

    HPF_pos_x.set_Time_constant(T_ude[0]);
    HPF_pos_y.set_Time_constant(T_ude[1]);
    HPF_pos_z.set_Time_constant(T_ude[2]);

    LPF_pos_error_x.set_Time_constant(T_ps);
    LPF_pos_error_y.set_Time_constant(T_ps);
    LPF_pos_error_z.set_Time_constant(T_ps);

    LPF_int_x.set_Time_constant(T_ude[0]);
    LPF_int_x.set_Time_constant(T_ude[1]);
    LPF_int_x.set_Time_constant(T_ude[2]);
}

prometheus_msgs::ControlOutput pos_controller_passivity::pos_controller(
    const prometheus_msgs::DroneState& _DroneState, 
    const prometheus_msgs::PositionReference& _Reference_State, float dt)
{
    Eigen::Vector3d accel_sp;

    // 计算误差项
    Eigen::Vector3f pos_error;
    
    pos_error = prometheus_control_utils::cal_pos_error(_DroneState, _Reference_State);

    // 误差项限幅
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
    }

    //z_k
    z_k[0] = HPF_pos_error_x.apply(pos_error[0], dt);
    z_k[1] = HPF_pos_error_y.apply(pos_error[1], dt);
    z_k[2] = HPF_pos_error_z.apply(pos_error[2], dt);

    //u_l
    for (int i = 0; i < 3; i++)
    {
       u_l[i] = _Reference_State.acceleration_ref[i] + (Kp[i] * pos_error[i] + Kd[i] * z_k[i]);
    }

    //UDE term y1 y2 y3
    y1_k[0] = HPF_pos_x.apply(_DroneState.position[0], dt);
    y1_k[1] = HPF_pos_y.apply(_DroneState.position[1], dt);
    y1_k[2] = HPF_pos_z.apply(_DroneState.position[2], dt);

    y2_k[0] = LPF_pos_error_x.apply(pos_error[0], dt);
    y2_k[1] = LPF_pos_error_y.apply(pos_error[1], dt);
    y2_k[2] = LPF_pos_error_z.apply(pos_error[2], dt);

    y3_k[0] = LPF_int_x.apply(_Reference_State.acceleration_ref[0] + Kp[0] * integral[0] + Kd[0] * y2_k[0], dt);
    y3_k[1] = LPF_int_y.apply(_Reference_State.acceleration_ref[1] + Kp[1] * integral[1] + Kd[1] * y2_k[1], dt);
    y3_k[2] = LPF_int_z.apply(_Reference_State.acceleration_ref[2] + Kp[2] * integral[2] + Kd[2] * y2_k[2], dt);

    for (int i = 0; i < 3; i++)
    {
        u_d[i] = y1_k[i] - y3_k[i];
    }

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        integral[i] += pos_error[i] * dt;

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
    }

    return _ControlOutput;
}


// 【打印参数函数】
void pos_controller_passivity::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>passivity Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;

    cout <<"Kp_X : "<< Kp[0] << endl;
    cout <<"Kp_Y : "<< Kp[1] << endl;
    cout <<"Kp_Z : "<< Kp[2] << endl;

    cout <<"Kd_X : "<< Kd[0] << endl;
    cout <<"Kd_Y : "<< Kd[1] << endl;
    cout <<"Kd_Z : "<< Kd[2] << endl;

    cout <<"passivity_T_X : "<< T_ude[0] << endl;
    cout <<"passivity_T_Y : "<< T_ude[1] << endl;
    cout <<"passivity_T_Z : "<< T_ude[2] << endl;
    cout <<"T_ps : "<< T_ps << endl;

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

void pos_controller_passivity::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>> Passivity Position Controller <<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << "z_k [X Y Z] : " << z_k[0] << " [N] "<< z_k[1]<<" [N] "<<z_k[2]<<" [N] "<<endl;

    cout << "y1 [X Y Z] : " << y1_k[0] << " [N] "<< y1_k[1]<<" [N] "<<y1_k[2]<<" [N] "<<endl;

    cout << "y2 [X Y Z] : " << y2_k[0] << " [N] "<< y2_k[1]<<" [N] "<<y2_k[2]<<" [N] "<<endl;

    cout << "y3 [X Y Z] : " << y3_k[0] << " [N] "<< y3_k[1]<<" [N] "<<y3_k[2]<<" [N] "<<endl;

    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;

    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;

}



#endif
