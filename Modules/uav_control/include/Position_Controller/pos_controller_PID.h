#ifndef POS_CONTROLLER_PID_H
#define POS_CONTROLLER_PID_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <prometheus_msgs/UAVState.h>

#include "math_utils.h"
#include "controller_utils.h"
#include "geometry_utils.h"
#include "printf_utils.h"

using namespace std;
 
class pos_controller_PID
{
    public:
        pos_controller_PID(){};

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

        void printf_param();
        void printf_result();
        Eigen::Vector4d update(float controller_hz);

    private:
        Ctrl_Param_PID ctrl_param;
        Desired_State desired_state;
        Current_State current_state;
        prometheus_msgs::UAVState uav_state; 
        Eigen::Vector3d F_des;

        Tracking_Error_Evaluation tracking_error;

        Eigen::Vector3d int_e_v;            // 积分
        Eigen::Quaterniond u_q_des;         // 期望姿态角（四元数）
        Eigen::Vector4d u_att;              // 期望姿态角（rad）+期望油门（0-1）
};


void pos_controller_PID::init(ros::NodeHandle& nh)
{
    // 【参数】控制参数
    ctrl_param.Kp.setZero();
    ctrl_param.Kv.setZero();
    ctrl_param.Kvi.setZero();
    ctrl_param.Ka.setZero();
    // 【参数】无人机质量
    nh.param<float>("pid_gain/quad_mass" , ctrl_param.quad_mass, 1.0f);
    // 【参数】悬停油门
    nh.param<float>("pid_gain/hov_percent" , ctrl_param.hov_percent, 0.5f);
    // 【参数】XYZ积分上限
    nh.param<float>("pid_gain/pxy_int_max"  , ctrl_param.int_max[0], 0.5);
    nh.param<float>("pid_gain/pxy_int_max"  , ctrl_param.int_max[1], 0.5);
    nh.param<float>("pid_gain/pz_int_max"   , ctrl_param.int_max[2], 0.5);
    // 【参数】控制参数
    nh.param<double>("pid_gain/Kp_xy", ctrl_param.Kp(0,0), 2.0f);
    nh.param<double>("pid_gain/Kp_xy", ctrl_param.Kp(1,1), 2.0f);
    nh.param<double>("pid_gain/Kp_z" , ctrl_param.Kp(2,2), 2.0f);
    nh.param<double>("pid_gain/Kv_xy", ctrl_param.Kv(0,0), 2.0f);
    nh.param<double>("pid_gain/Kv_xy", ctrl_param.Kv(1,1), 2.0f);
    nh.param<double>("pid_gain/Kv_z" , ctrl_param.Kv(2,2), 2.0f);
    nh.param<double>("pid_gain/Kvi_xy", ctrl_param.Kvi(0,0), 0.3f);
    nh.param<double>("pid_gain/Kvi_xy", ctrl_param.Kvi(1,1), 0.3f);
    nh.param<double>("pid_gain/Kvi_z" , ctrl_param.Kvi(2,2), 0.3f);
    nh.param<float>("pid_gain/tilt_angle_max" , ctrl_param.tilt_angle_max, 10.0f);
    ctrl_param.g << 0.0, 0.0, 9.8;

    printf_param();
}

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
// 期望姿态 + 期望油门
Eigen::Vector4d pos_controller_PID::update(float controller_hz)
{
    // 定点控制的时候才积分，即追踪轨迹或者速度追踪时不进行积分
	if (desired_state.vel(0) != 0.0 || desired_state.vel(1) != 0.0 || desired_state.vel(2) != 0.0) 
    {
        PCOUT(2, YELLOW, "Reset integration.");
		int_e_v.setZero();
	}

    // 位置误差
    Eigen::Vector3d pos_error = desired_state.pos - current_state.pos;
    Eigen::Vector3d vel_error = desired_state.vel - current_state.vel;
    
    tracking_error.input_error(pos_error,vel_error);

    // 限制最大误差
    float max_pos_error = 3.0;
    float max_vel_error = 3.0;

    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) > max_pos_error)
        {            
            pos_error[i] = (pos_error[i] > 0) ? 1.0 : -1.0;
        }
        if(abs(vel_error[i]) > max_vel_error)
        {            
            vel_error[i] = (vel_error[i] > 0) ? 2.0 : -2.0;
        }
    }

    // 积分项 - XY
    for (int i=0; i<2; i++)
    {
        // 只有在pos_error比较小时，才会启动积分
        float int_start_error = 0.2;
        if(abs(pos_error[i]) < int_start_error && uav_state.mode == "OFFBOARD")
        {
            int_e_v[i] += pos_error[i] / controller_hz;
            if(abs(int_e_v[i]) > ctrl_param.int_max[i])
            {
                PCOUT(2, YELLOW, "int_e_v saturation [ xy ]");
                int_e_v[i] = (int_e_v[i] > 0) ? ctrl_param.int_max[i] : -ctrl_param.int_max[i];
            }
        }else
        {
            int_e_v[i] = 0;
        }
    }

    // 积分项 - Z
    float int_start_error = 0.5;
    if(abs(pos_error[2]) < int_start_error && uav_state.mode == "OFFBOARD")
    {
        int_e_v[2] += pos_error[2] / controller_hz;

        if(abs(int_e_v[2]) > ctrl_param.int_max[2])
        {
            PCOUT(2, YELLOW, "int_e_v saturation [ z ]");
            int_e_v[2] = (int_e_v[2] > 0) ? ctrl_param.int_max[2] : -ctrl_param.int_max[2];
        }
    }else
    {
        int_e_v[2] = 0;
    }

    // 期望加速度 = 期望加速度 + Kp * 位置误差 + Kv * 速度误差 + Kv * 积分项
    Eigen::Vector3d des_acc = desired_state.acc + ctrl_param.Kp * pos_error + ctrl_param.Kv * vel_error + ctrl_param.Kvi* int_e_v;

	// 期望力 = 质量*控制量 + 重力抵消
    // F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
    // u_att是用于PX4的姿态控制输入，u_att 前三位是roll pitch yaw， 第四位为油门值[0-1]
    F_des = des_acc * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g;

	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * ctrl_param.quad_mass * ctrl_param.g(2))
	{
		F_des = F_des / F_des(2) * (0.5 * ctrl_param.quad_mass * ctrl_param.g(2));
	}
	else if (F_des(2) > 2 * ctrl_param.quad_mass * ctrl_param.g(2))
	{
		F_des = F_des / F_des(2) * (2 * ctrl_param.quad_mass * ctrl_param.g(2));
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
        PCOUT(2, YELLOW, "pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
        PCOUT(2, YELLOW, "roll too tilt");
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
        PCOUT(2, YELLOW, "throttle too low");
    }

    if(u_att(3) > 1.0)
    {
        u_att(3) = 1.0;
        PCOUT(2, YELLOW, "throttle too high");
    }

    return u_att;
}

void pos_controller_PID::printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    cout<<setprecision(2);
    cout << BLUE << "----> PID Position Controller Debug Info      : " << TAIL << endl;
    cout << BLUE << "----> pos_des         : " << desired_state.pos(0) << " [ m ] " << desired_state.pos(1) << " [ m ] " << desired_state.pos(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> vel_des         : " << desired_state.vel(0) << " [ m ] " << desired_state.vel(1) << " [ m ] " << desired_state.vel(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> acc_des         : " << desired_state.acc(0) << " [ m ] " << desired_state.acc(1) << " [ m ] " << desired_state.acc(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> pos_now         : " << current_state.pos(0) << " [ m ] " << current_state.pos(1) << " [ m ] " << current_state.pos(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> vel_now         : " << current_state.vel(0) << " [ m ] " << current_state.vel(1) << " [ m ] " << current_state.vel(2) << " [ m ] "<< TAIL << endl;
    
    cout << BLUE << "----> int_e_v         : " << int_e_v(0) << " [N] "<< int_e_v(1) << " [N] "<< int_e_v(2) << " [N] "<< TAIL << endl;
    
    cout << BLUE << "----> F_des           : " << F_des(0) << " [N] "<< F_des(1) << " [N] "<< F_des(2) << " [N] "<< TAIL << endl;
    
    cout << BLUE << "----> u_attitude      : " << u_att(0)*180/3.14 << " [deg] "<< u_att(1)*180/3.14 << " [deg] "<< u_att(2)*180/3.14 << " [deg] "<< TAIL << endl;
    cout << BLUE << "----> u_throttle      : " << u_att(3) << " [0-1] "<< TAIL << endl;
    cout << BLUE << "----> pos_error_mean  : " << tracking_error.pos_error_mean <<" [m] "<< TAIL <<endl;
    cout << BLUE << "----> vel_error_mean  : " << tracking_error.vel_error_mean <<" [m/s] "<< TAIL <<endl;
}

// 【打印参数函数】
void pos_controller_PID::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl;
    cout << GREEN <<  "ctrl_param.quad_mass     : "<< ctrl_param.quad_mass<< TAIL <<endl;
    cout << GREEN <<  "ctrl_param.hov_percent   : "<< ctrl_param.hov_percent<< TAIL <<endl;
    cout << GREEN <<  "pxy_int_max              : "<< ctrl_param.int_max[0]<< TAIL <<endl;
    cout << GREEN <<  "pz_int_max               : "<< ctrl_param.int_max[2]<< TAIL <<endl;

    cout << GREEN <<  "Kp_xy         : "<< ctrl_param.Kp(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kp_z          : "<< ctrl_param.Kp(2,2) << TAIL <<endl;
    cout << GREEN <<  "Kv_xy         : "<< ctrl_param.Kv(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kv_z          : "<< ctrl_param.Kv(2,2) << TAIL <<endl;
    cout << GREEN <<  "Kvi_xy        : "<< ctrl_param.Kvi(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kvi_z         : "<< ctrl_param.Kvi(2,2) << TAIL <<endl;
    cout << GREEN <<  "Ka_xy         : "<< ctrl_param.Ka(0,0) << TAIL <<endl;
    cout << GREEN <<  "Ka_z          : "<< ctrl_param.Ka(2,2) << TAIL <<endl;
    cout << GREEN <<  "tilt_angle_max: "<< ctrl_param.tilt_angle_max << TAIL <<endl;


}
#endif