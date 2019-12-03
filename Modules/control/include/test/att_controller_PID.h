/***************************************************************************************************************************
* att_controller_PID.h
*
* Author: Qyp
*
* Update Time: 2019.3.23
*
* Introduction:  attition Controller using PID (P for att loop, pid for rates loop)
*         1. Similiar to the attition controller in PX4 (1.8.2)
*         2. Ref to : https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
*                       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
*         3. Here we didn't consider the mass of the drone, we treat accel_sp is the thrust_sp.
*         4. We didn't use filter to get the derivation of the ratesocity [It must be considered.]
*         5. ThrottleToAttitude ref to https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/Utility/ControlMath.cpp
***************************************************************************************************************************/
#ifndef ATT_CONTROLLER_PID_H
#define ATT_CONTROLLER_PID_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>


using namespace std;

namespace namespace_PID {

class att_controller_PID
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        att_controller_PID(void):
            att_pid_nh("~")
        {
            att_pid_nh.param<float>("MC_ROLL_P", MC_ROLL_P, 6.5);
            att_pid_nh.param<float>("MC_ROLLRATE_P", MC_ROLLRATE_P, 0.15);
            att_pid_nh.param<float>("MC_ROLLRATE_I", MC_ROLLRATE_I, 0.05);
            att_pid_nh.param<float>("MC_ROLLRATE_D", MC_ROLLRATE_D, 0.003);
            att_pid_nh.param<float>("MC_RR_INT_LIM", MC_RR_INT_LIM, 0.3);

            att_pid_nh.param<float>("MC_PITCH_P", MC_PITCH_P, 6.5);
            att_pid_nh.param<float>("MC_PITCHRATE_P", MC_PITCHRATE_P, 0.15);
            att_pid_nh.param<float>("MC_PITCHRATE_I", MC_PITCHRATE_I, 0.05);
            att_pid_nh.param<float>("MC_PITCHRATE_D", MC_PITCHRATE_D, 0.003);
            att_pid_nh.param<float>("MC_PR_INT_LIM", MC_PR_INT_LIM, 0.3);

            att_pid_nh.param<float>("MC_YAW_P", MC_YAW_P, 2.8);
            att_pid_nh.param<float>("MC_YAWRATE_P", MC_YAWRATE_P, 0.2);
            att_pid_nh.param<float>("MC_YAWRATE_I", MC_YAWRATE_I, 0.1);
            att_pid_nh.param<float>("MC_YAWRATE_D", MC_YAWRATE_D, 0.00);
            att_pid_nh.param<float>("MC_YR_INT_LIM", MC_YR_INT_LIM, 0.3);
            att_pid_nh.param<float>("MC_YAW_FF", MC_YAW_FF, 0.5);

            att_pid_nh.param<float>("MC_ROLLRATE_MAX", MC_ROLLRATE_MAX, 220.0);
            att_pid_nh.param<float>("MC_PITCHRATE_MAX", MC_PITCHRATE_MAX, 220.0);
            att_pid_nh.param<float>("MC_YAWRATE_MAX", MC_YAWRATE_MAX, 200.0);
            att_pid_nh.param<float>("MC_DTERM_CUTOFF", MC_DTERM_CUTOFF, 30.0);



            Euler_fcu       = Eigen::Vector3d(0.0,0.0,0.0);
            q_fcu           = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
            rates_fcu       = Eigen::Vector3d(0.0,0.0,0.0);
            rates_P_output       = Eigen::Vector3d(0.0,0.0,0.0);
            rates_int       = Eigen::Vector3d(0.0,0.0,0.0);
            rates_D_output       = Eigen::Vector3d(0.0,0.0,0.0);
            error_rates_last       = Eigen::Vector3d(0.0,0.0,0.0);
            euler_setpoint       = Eigen::Vector3d(0.0,0.0,0.0);
            rates_setpoint       = Eigen::Vector3d(0.0,0.0,0.0);
            actuator_setpoint       = Eigen::Vector4d(0.0,0.0,0.0,0.0);

            thrust_sp       = Eigen::Vector3d(0.0,0.0,0.0);

            flag_offboard   = 0;

            state_sub = att_pid_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &att_controller_PID::state_cb,this);
        }

        //PID parameter for the control law
        float MC_ROLL_P;
        float MC_ROLLRATE_P;
        float MC_ROLLRATE_I;
        float MC_ROLLRATE_D;
        float MC_RR_INT_LIM;

        float MC_PITCH_P;
        float MC_PITCHRATE_P;
        float MC_PITCHRATE_I;
        float MC_PITCHRATE_D;
        float MC_PR_INT_LIM;

        float MC_YAW_P;
        float MC_YAWRATE_P;
        float MC_YAWRATE_I;
        float MC_YAWRATE_D;
        float MC_YR_INT_LIM;
        float MC_YAW_FF;

        float MC_ROLLRATE_MAX;
        float MC_PITCHRATE_MAX;
        float MC_YAWRATE_MAX;
        float MC_DTERM_CUTOFF;


        //Current att of the drone
        Eigen::Quaterniond q_fcu;
        Eigen::Vector3d Euler_fcu;
        Eigen::Vector3d rates_fcu;

        //Output of the rates loop in PID [rates_int is the I]
        Eigen::Vector3d rates_P_output;
        Eigen::Vector3d rates_int;
        Eigen::Vector3d rates_D_output;
        //Error of the rates in last step [used for the D-output in rates loop]
        Eigen::Vector3d error_rates_last;
        Eigen::Vector3d error_rates_dot_last;
\


        //
        Eigen::Vector3d euler_setpoint;
        Eigen::Vector3d rates_setpoint;
        Eigen::Vector4d actuator_setpoint;




        //The delta time between now and the last step
        float delta_time;
        //Time of the last step
        float last_time;


        Eigen::Vector3d thrust_sp;

        float thrust_body_sp;

        //Current state of the drone
        mavros_msgs::State current_state;

        //Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD mode]
        int flag_offboard;




        //Printf the PID parameter
        void printf_pid_param();

        //Printf the control result
        void printf_result();

        //attition control main function [Input: desired state, time_now; Output: actuator_setpoint;]
        Eigen::Vector4d att_controller(Eigen::Vector3d att, Eigen::Vector3d rates, Eigen::Vector3d accel_sp, float yaw_sp, float curtime);

        //ThrottleToAttitude [Input: desired thrust,desired yaw angle; Output: desired euler angle]
        void ThrottleToAttitude(float yaw_sp);

        //attition control loop [Input: desired state; Output: desired rates]
        void _attController();

        //ratesocity control loop [Input: rates_setpoint; Output: actuator_setpoint]
        void _attrateController();

        Eigen::Vector3d cal_rates_error_deriv(Eigen::Vector3d error_now);

    private:

        ros::NodeHandle att_pid_nh;

        ros::Subscriber state_sub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            current_state = *msg;

            if(current_state.mode == "OFFBOARD")
            {
                flag_offboard = 1;
            }else
            {
                flag_offboard = 0;
            }

        }

};


//attition control main function [Input: desired state, time_now; Output: actuator_setpoint;]
Eigen::Vector4d att_controller_PID::att_controller(Eigen::Vector3d att, Eigen::Vector3d rates, Eigen::Vector3d accel_sp, float yaw_sp, float curtime)
{
    delta_time = curtime - last_time;

    thrust_sp = accel_sp;

    Euler_fcu = att;

    rates_fcu = rates;

    ThrottleToAttitude(yaw_sp);

    _attController();

    _attrateController();

    last_time = curtime;

    return actuator_setpoint;
}

void att_controller_PID::ThrottleToAttitude(float yaw_sp)
{

    Eigen::Vector3d body_x,body_y,body_z;

    euler_setpoint(2) = yaw_sp;

    float thrust_sp_length = thrust_sp.norm();

    if (thrust_sp_length > 0.00001f) {

        //ENU or NED is different
            //body_z = -thrust_sp.normalized();
        body_z = thrust_sp.normalized();

    } else {
            // no thrust, set Z axis to safe value
            body_z(0) = 0.0f;
            body_z(1) = 0.0f;
            body_z(2) = 1.0f;
    }

    // vector of desired yaw direction in XY plane, rotated by PI/2
    Eigen::Vector3d y_C(-sin(euler_setpoint(2)), cos(euler_setpoint(2)), 0.0f);

    if (fabs(body_z(2)) > 0.000001f) {
            // desired body_x axis, orthogonal to body_z
            body_x = y_C.cross( body_z);

            // keep nose to front while inverted upside down
            if (body_z(2) < 0.0f) {
                    body_x = -body_x;
            }

            body_x.normalize();

    } else {
            // desired thrust is in XY plane, set X downside to construct correct matrix,
            // but yaw component will not be used actually
            body_x(0) = 0.0f;
            body_x(1) = 0.0f;
            body_x(2) = 1.0f;
    }

        // desired body_y axis
        body_y = body_z .cross( body_x );

        Eigen::Matrix3d R_sp;

        // fill rotation matrix
        for (int i = 0; i < 3; i++) {
                R_sp(i, 0) = body_x(i);
                R_sp(i, 1) = body_y(i);
                R_sp(i, 2) = body_z(i);
        }

        //这里，由于NED与ENU的关系 ，导致结算出来的欧拉角度有相差
        euler_setpoint = rotation_to_euler(R_sp);

        thrust_body_sp = thrust_sp_length;
}




void att_controller_PID::_attController()
{
    rates_setpoint(0) = MC_ROLL_P  * (euler_setpoint(0) - Euler_fcu(0));
    rates_setpoint(1) = MC_PITCH_P * (euler_setpoint(1) - Euler_fcu(1));
    rates_setpoint(2) = MC_YAW_P   * (euler_setpoint(2) - Euler_fcu(2));

    // Limit the ratesocity setpoint
    rates_setpoint(0) = constrain_function2(rates_setpoint(0), -MC_ROLLRATE_MAX,  MC_ROLLRATE_MAX);
    rates_setpoint(1) = constrain_function2(rates_setpoint(1), -MC_PITCHRATE_MAX, MC_PITCHRATE_MAX);
    rates_setpoint(2) = constrain_function2(rates_setpoint(2), -MC_YAWRATE_MAX,   MC_YAWRATE_MAX);
}

void att_controller_PID::_attrateController()
{
    Eigen::Vector3d error_rates = rates_setpoint - rates_fcu;

    rates_P_output(0) = MC_ROLLRATE_P  * error_rates(0);
    rates_P_output(1) = MC_PITCHRATE_P * error_rates(1);
    rates_P_output(2) = MC_YAWRATE_P   * error_rates(2);

    Eigen::Vector3d rates_error_deriv = cal_rates_error_deriv(error_rates);

    rates_D_output(0) = MC_ROLLRATE_D  * rates_error_deriv(0);
    rates_D_output(1) = MC_PITCHRATE_D * rates_error_deriv(1);
    rates_D_output(2) = MC_YAWRATE_D   * rates_error_deriv(2);

    // Update integral
    rates_int(0) += MC_ROLLRATE_I  * error_rates(0) * delta_time;
    rates_int(1) += MC_PITCHRATE_I * error_rates(1) * delta_time;
    rates_int(2) += MC_YAWRATE_I   * error_rates(2) * delta_time;

    rates_int(0) = constrain_function2(rates_int(0), -MC_RR_INT_LIM, MC_RR_INT_LIM);
    rates_int(1) = constrain_function2(rates_int(1), -MC_PR_INT_LIM, MC_PR_INT_LIM);
    rates_int(2) = constrain_function2(rates_int(2), -MC_YR_INT_LIM, MC_YR_INT_LIM);

    //
    actuator_setpoint(0) = rates_P_output(0) + rates_int(0) + rates_D_output(0);
    actuator_setpoint(1) = rates_P_output(1) + rates_int(1) + rates_D_output(1);
    actuator_setpoint(2) = rates_P_output(2) + rates_int(2) + rates_D_output(2);
    actuator_setpoint(3) = thrust_body_sp;

    //If not in OFFBOARD mode, set all intergral to zero.
    if(flag_offboard == 0)
    {
        rates_int = Eigen::Vector3d(0.0,0.0,0.0);
    }

}

Eigen::Vector3d att_controller_PID::cal_rates_error_deriv(Eigen::Vector3d error_now)
{
    Eigen::Vector3d error_rates_dot_now;
    error_rates_dot_now = (error_now - error_rates_last)/delta_time;

    error_rates_last = error_now;
    float a,b;
    b = 2 * M_PI * MC_DTERM_CUTOFF * delta_time;
    a = b / (1 + b);

    Eigen::Vector3d output;

    output = a * error_rates_dot_now + (1 - a) * error_rates_dot_last ;

    error_rates_dot_last = output;

    return output;
}

void att_controller_PID::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>attition Controller<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << "delta_time : " << delta_time<< " [s] " <<endl;

    cout << "euler_setpoint [X Y Z] : " << euler_setpoint[0] / M_PI *180 << " [deg] "<< euler_setpoint[1] / M_PI *180 <<" [deg] "<<euler_setpoint[2]/ M_PI *180 <<" [deg] "<<endl;

    cout << "rates_setpoint [X Y Z] : " << rates_setpoint[0] / M_PI *180 << " [deg/s] "<< rates_setpoint[1] / M_PI *180 <<" [deg/s] "<<rates_setpoint[2]/ M_PI *180 <<" [deg/s] "<<endl;

    cout << "rates_P_output [X Y Z] : " << rates_P_output[0] << " [m/s] "<< rates_P_output[1]<<" [m/s] "<<rates_P_output[2]<<" [m/s] "<<endl;

    cout << "rates_I_output [X Y Z] : " << rates_int[0] << " [m/s] "<< rates_int[1]<<" [m/s] "<<rates_int[2]<<" [m/s] "<<endl;

    cout << "rates_D_output [X Y Z] : " << rates_D_output[0] << " [m/s] "<< rates_D_output[1]<<" [m/s] "<<rates_D_output[2]<<" [m/s] "<<endl;

    cout << "actuator_setpoint [0 1 2 3] : " << actuator_setpoint(0) << " [ ] "<< actuator_setpoint(1) <<" [ ] "<< actuator_setpoint(2) <<" [ ] "<< actuator_setpoint(3)<<" [ ] "<<endl;
}

// 【打印参数函数】
void att_controller_PID::printf_pid_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>> PID Parameter for Attitude Control <<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"MC_ROLL_P : "<< MC_ROLL_P << endl;
    cout <<"MC_ROLLRATE_P : "<< MC_ROLLRATE_P << endl;
    cout <<"MC_ROLLRATE_I : "<< MC_ROLLRATE_I << endl;
    cout <<"MC_ROLLRATE_D : "<< MC_ROLLRATE_D << endl;

    cout <<"MC_PITCH_P : "<< MC_PITCH_P << endl;
    cout <<"MC_PITCHRATE_P : "<< MC_PITCHRATE_P << endl;
    cout <<"MC_PITCHRATE_I : "<< MC_PITCHRATE_I << endl;
    cout <<"MC_PITCHRATE_D : "<< MC_PITCHRATE_D << endl;

    cout <<"MC_YAW_P : "<< MC_YAW_P << endl;
    cout <<"MC_YAWRATE_P : "<< MC_YAWRATE_P << endl;
    cout <<"MC_YAWRATE_I : "<< MC_YAWRATE_I << endl;
    cout <<"MC_YAWRATE_D : "<< MC_YAWRATE_D << endl;


    cout <<"MC_ROLLRATE_MAX : "<< MC_ROLLRATE_MAX << endl;
    cout <<"MC_PITCHRATE_MAX : "<< MC_PITCHRATE_MAX << endl;
    cout <<"MC_YAWRATE_MAX : "<< MC_YAWRATE_MAX << endl;



    cout <<"MC_RR_INT_LIM : "<< MC_RR_INT_LIM << endl;
    cout <<"MC_PR_INT_LIM : "<< MC_PR_INT_LIM << endl;
    cout <<"MC_YR_INT_LIM : "<< MC_YR_INT_LIM << endl;


}



}
#endif
