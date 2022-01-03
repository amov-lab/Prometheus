#ifndef POS_CONTROLLER_UDE_H
#define POS_CONTROLLER_UDE_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <bitset>
#include <prometheus_msgs/UAVState.h>

#include "geometry_utils.h"
#include "controller_utils.h"
#include "printf_utils.h"

using namespace std;

class pos_controller_UDE
{
    public:
        pos_controller_UDE(){};

        void init(ros::NodeHandle& nh);

        void set_desired_state(const Desired_State& des)
        {
            desired_state = des;
        }

        void set_current_state(const prometheus_msgs::UAVState& state)
        {
            uav_state = state;

            for(int i=0; i<3; i++)
            {
                current_state.pos(i) = uav_state.position[i];
                current_state.vel(i) = uav_state.velocity[i];
            }

            current_state.q.w() = uav_state.attitude_q.w;
            current_state.q.x() = uav_state.attitude_q.x;
            current_state.q.y() = uav_state.attitude_q.y;
            current_state.q.z() = uav_state.attitude_q.z; 

            current_state.yaw = geometry_utils::get_yaw_from_quaternion(current_state.q);
        }

        Eigen::Vector4d update(float controller_hz);

    private:

        Ctrl_Param_UDE ctrl_param;
        Desired_State desired_state;
        Current_State current_state;
        prometheus_msgs::UAVState uav_state;

        Tracking_Error_Evaluation tracking_error;

        //u_l for nominal contorol(PD), u_d for ude control(disturbance estimator)
        Eigen::Vector3d u_l,u_d;
        Eigen::Vector3d integral;
        Eigen::Vector3d F_des;
        Eigen::Vector4d u_att;                  // 期望姿态角（rad）+期望油门（0-1）
        
        void printf_param();
        void printf_result();
};


void pos_controller_UDE::init(ros::NodeHandle& nh)
{
    ctrl_param.Kp.setZero();
    ctrl_param.Kd.setZero();
    nh.param<float>("ude_gain/quad_mass"   , ctrl_param.quad_mass, 1.0f);
    nh.param<float>("ude_gain/hov_percent" , ctrl_param.hov_percent, 0.5f);
    nh.param<float>("ude_gain/pxy_int_max" , ctrl_param.int_max[0], 1.0);
    nh.param<float>("ude_gain/pxy_int_max" , ctrl_param.int_max[1], 1.0);
    nh.param<float>("ude_gain/pz_int_max"  , ctrl_param.int_max[2], 1.0);
    ctrl_param.g << 0.0, 0.0, 9.8;

    nh.param<double>("ude_gain/Kp_xy", ctrl_param.Kp(0,0), 0.5f);
    nh.param<double>("ude_gain/Kp_xy", ctrl_param.Kp(1,1), 0.5f);
    nh.param<double>("ude_gain/Kp_z" , ctrl_param.Kp(2,2), 0.5f);
    nh.param<double>("ude_gain/Kd_xy", ctrl_param.Kd(0,0), 2.0f);
    nh.param<double>("ude_gain/Kd_xy", ctrl_param.Kd(1,1), 2.0f);
    nh.param<double>("ude_gain/Kd_z" , ctrl_param.Kd(2,2), 2.0f);
    nh.param<double>("ude_gain/T_ude", ctrl_param.T_ude, 1.0f);
    nh.param<float>("ude_gain/tilt_angle_max" , ctrl_param.tilt_angle_max, 20.0f);

    u_l.setZero();
    u_d.setZero();
    integral.setZero();
    u_att.setZero();

    printf_param();
}

Eigen::Vector4d pos_controller_UDE::update(float controller_hz)
{
    float dt = 1/controller_hz;
    // 位置误差
    Eigen::Vector3d pos_error = desired_state.pos - current_state.pos;
    // 速度误差
    Eigen::Vector3d vel_error = desired_state.vel - current_state.vel;
    // 误差评估
    tracking_error.input_error(pos_error,vel_error);
    
    // 限制最大位置误差
    float max_pos_error = 3.0;
    float max_vel_error = 3.0;

    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) > max_pos_error)
        {            
            pos_error[i] = (pos_error[i] > 0) ? max_pos_error : -max_pos_error;
        }

        if(abs(vel_error[i]) > max_vel_error)
        {            
            vel_error[i] = (vel_error[i] > 0) ? max_vel_error : -max_vel_error;
        }

    }

    // UDE算法
    u_l = desired_state.acc + ctrl_param.Kp * pos_error + ctrl_param.Kd * vel_error;
    u_d = - 1.0 / ctrl_param.T_ude * (ctrl_param.Kp * integral + ctrl_param.Kd * pos_error + vel_error);

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        float int_start_error = 0.5;
        if(abs(pos_error[i]) < int_start_error)
        {
            integral[i] += pos_error[i] * dt;
        }else
        {
            integral[i] = 0;
        }

        if(abs(u_d[i]) > ctrl_param.int_max[i])
        {
            cout << "u_d saturation! " << " [0-1-2] "<< i <<endl;
            cout << "[u_d]: "<< u_d[i]<<" [u_d_max]: "<<ctrl_param.int_max[i]<<" [m/s] "<<endl;
            u_d[i] = (u_d[i] > 0) ? ctrl_param.int_max[i] : -ctrl_param.int_max[i];
        }
    }

    // 期望加速度
    Eigen::Vector3d u_v = u_l - u_d;

	// 期望力 = 质量*控制量 + 重力抵消
	F_des = u_v * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g;
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * ctrl_param.quad_mass * ctrl_param.g(2))
	{
        ROS_INFO("F_des too low");
		F_des = F_des / F_des(2) * (0.5 * ctrl_param.quad_mass * ctrl_param.g(2));
	}
	else if (F_des(2) > 2 * ctrl_param.quad_mass * ctrl_param.g(2))
	{
        ROS_INFO("F_des too high");
		F_des = F_des / F_des(2) * (2 * ctrl_param.quad_mass * ctrl_param.g(2));
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
		ROS_INFO("pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
		ROS_INFO("roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = geometry_utils::rotz(current_state.yaw);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1)  = std::atan2( fx, fz);
    u_att(2)  = desired_state.yaw; 

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = current_state.q.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的电机推力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = ctrl_param.quad_mass * ctrl_param.g(2) / ctrl_param.hov_percent;

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    u_att(3) = u1 / full_thrust;

    if(u_att(3) < 0.1)
    {
        u_att(3) = 0.1;
        ROS_INFO("throttle too low");
    }

    if(u_att(3) > 1.0)
    {
        u_att(3) = 1.0;
        ROS_INFO("throttle too high");
    }

    return u_att;
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
    
    cout << "u_l [X Y Z]    : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;
    cout << "int [X Y Z]    : " << integral[0] << " [N] "<< integral[1]<<" [N] "<<integral[2]<<" [N] "<<endl;
    cout << "u_d [X Y Z]    : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;
    cout << "F_des [X Y Z]  : " << F_des[0] << " [N] "<< F_des[1]<<" [N] "<<F_des[2]<<" [N] "<<endl;
    cout << "u_att [X Y Z]  : " << u_att[0] << " [rad] "<< u_att[1]<<" [rad] "<<u_att[2]<<" [rad] "<<endl;
    cout << "u_throttle  : " << u_att[3] <<endl;
    cout << "pos_error_mean : " << tracking_error.pos_error_mean <<" [m] "<<endl;
    cout << "vel_error_mean : " << tracking_error.vel_error_mean <<" [m/s] "<<endl;

}

// 【打印参数函数】
void pos_controller_UDE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>UDE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "ctrl_param.quad_mass   : "<< ctrl_param.quad_mass<<endl;
    cout << "ctrl_param.hov_percent   : "<< ctrl_param.hov_percent<<endl;
    cout << "pxy_int_max   : "<< ctrl_param.int_max[0]<<endl;
    cout << "pz_int_max   : "<< ctrl_param.int_max[2]<<endl;

    cout << "ude_gain/Kp_xy   : "<< ctrl_param.Kp(0,0) <<endl;
    cout << "ude_gain/Kp_z    : "<< ctrl_param.Kp(2,2) <<endl;
    cout << "ude_gain/Kd_xy   : "<< ctrl_param.Kd(0,0) <<endl;
    cout << "ude_gain/Kd_z    : "<< ctrl_param.Kd(2,2) <<endl;
    cout << "ude_gain/T_ude   : "<< ctrl_param.T_ude <<endl;
    cout << "ude_gain/tilt_angle_max    : "<< ctrl_param.tilt_angle_max <<endl;

}


#endif
