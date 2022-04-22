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

        Eigen::Vector4d update(float controller_hz);

    private:

        Ctrl_Param_PID ctrl_param_hover;
        Ctrl_Param_PID ctrl_param_track;
        Ctrl_Param_PID ctrl_param;
        Desired_State desired_state;
        Current_State current_state;
        prometheus_msgs::UAVState uav_state; 

        Tracking_Error_Evaluation tracking_error;

        Eigen::Vector3d int_e_v;            // 积分
        Eigen::Quaterniond u_q_des;         // 期望姿态角（四元数）
        Eigen::Vector4d u_att;              // 期望姿态角（rad）+期望油门（0-1）

        void printf_param();
        void printf_result();
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
    ctrl_param.tilt_angle_max = 10.0;
    ctrl_param.g << 0.0, 0.0, 9.8;
    ctrl_param_hover = ctrl_param;
    ctrl_param_track = ctrl_param;

    // 【参数】定点控制参数
    nh.param<double>("pid_gain/hover_gain/Kp_xy", ctrl_param_hover.Kp(0,0), 2.0f);
    nh.param<double>("pid_gain/hover_gain/Kp_xy", ctrl_param_hover.Kp(1,1), 2.0f);
    nh.param<double>("pid_gain/hover_gain/Kp_z" , ctrl_param_hover.Kp(2,2), 2.0f);
    nh.param<double>("pid_gain/hover_gain/Kv_xy", ctrl_param_hover.Kv(0,0), 2.0f);
    nh.param<double>("pid_gain/hover_gain/Kv_xy", ctrl_param_hover.Kv(1,1), 2.0f);
    nh.param<double>("pid_gain/hover_gain/Kv_z" , ctrl_param_hover.Kv(2,2), 2.0f);
    nh.param<double>("pid_gain/hover_gain/Kvi_xy", ctrl_param_hover.Kvi(0,0), 0.3f);
    nh.param<double>("pid_gain/hover_gain/Kvi_xy", ctrl_param_hover.Kvi(1,1), 0.3f);
    nh.param<double>("pid_gain/hover_gain/Kvi_z" , ctrl_param_hover.Kvi(2,2), 0.3f);
    nh.param<double>("pid_gain/hover_gain/Ka_xy", ctrl_param_hover.Ka(0,0), 1.0f);
    nh.param<double>("pid_gain/hover_gain/Ka_xy", ctrl_param_hover.Ka(1,1), 1.0f);
    nh.param<double>("pid_gain/hover_gain/Ka_z" , ctrl_param_hover.Ka(2,2), 1.0f);
    nh.param<float>("pid_gain/hover_gain/tilt_angle_max" , ctrl_param_hover.tilt_angle_max, 10.0f);

    // 【参数】轨迹追踪参数
    nh.param<double>("pid_gain/track_gain/Kp_xy", ctrl_param_track.Kp(0,0), 3.0f);
    nh.param<double>("pid_gain/track_gain/Kp_xy", ctrl_param_track.Kp(1,1), 3.0f);
    nh.param<double>("pid_gain/track_gain/Kp_z" , ctrl_param_track.Kp(2,2), 3.0f);
    nh.param<double>("pid_gain/track_gain/Kv_xy", ctrl_param_track.Kv(0,0), 3.0f);
    nh.param<double>("pid_gain/track_gain/Kv_xy", ctrl_param_track.Kv(1,1), 3.0f);
    nh.param<double>("pid_gain/track_gain/Kv_z" , ctrl_param_track.Kv(2,2), 3.0f);
    nh.param<double>("pid_gain/track_gain/Kvi_xy", ctrl_param_track.Kvi(0,0), 0.1f);
    nh.param<double>("pid_gain/track_gain/Kvi_xy", ctrl_param_track.Kvi(1,1), 0.1f);
    nh.param<double>("pid_gain/track_gain/Kvi_z" , ctrl_param_track.Kvi(2,2), 0.1f);
    nh.param<double>("pid_gain/track_gain/Ka_xy", ctrl_param_track.Ka(0,0), 1.0f);
    nh.param<double>("pid_gain/track_gain/Ka_xy", ctrl_param_track.Ka(1,1), 1.0f);
    nh.param<double>("pid_gain/track_gain/Ka_z" , ctrl_param_track.Ka(2,2), 1.0f);
    nh.param<float>("pid_gain/track_gain/tilt_angle_max" , ctrl_param_track.tilt_angle_max, 20.0f);

    printf_param();
}

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
Eigen::Vector4d pos_controller_PID::update(float controller_hz)
{
       
    // 定点控制的时候才积分，即追踪轨迹或者速度追踪时不进行积分
	if (desired_state.vel(0) != 0.0 || desired_state.vel(1) != 0.0 || desired_state.vel(2) != 0.0) 
    {
		//ROS_INFO("Reset integration");
		int_e_v.setZero();
	}

    // 位置误差
    Eigen::Vector3d pos_error = desired_state.pos - current_state.pos;
    Eigen::Vector3d vel_error = desired_state.vel - current_state.vel;
    
    tracking_error.input_error(pos_error,vel_error);

    // 限制最大位置误差
    float max_pos_error = 1.0;
    float max_vel_error = 2.0;
    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) > max_pos_error)
        {            
            pos_error[i] = (pos_error[i] > 0) ? 1.0 : -1.0;
        }
    }

    // 位置控制量
    Eigen::Vector3d u_pos = ctrl_param.Kp * pos_error;

    // 速度误差
    vel_error = vel_error + u_pos;

    // 限制最大位置误差
    for (int i=0; i<3; i++)
    {
        if(abs(vel_error[i]) > max_vel_error)
        {            
            vel_error[i] = (vel_error[i] > 0) ? 2.0 : -2.0;
        }
    }

    // 速度控制量
    Eigen::Vector3d u_vel = ctrl_param.Kv * vel_error;  

    // 积分项
    for (int i=0; i<2; i++)
    {
        // 只有在pos_error比较小时，才会启动积分
        float int_start_error = 0.2;
        if(abs(pos_error[i]) < int_start_error && uav_state.mode == "OFFBOARD")
        {
            int_e_v[i] += pos_error[i] / controller_hz;

            if(abs(int_e_v[i]) > ctrl_param.int_max[i])
            {
                // cout << YELLOW << "----> int_e_v saturation [ "<< i << " ]"<<" [ctrl_param.int_max]: "<<ctrl_param.int_max[i]<<" [m/s] "<< TAIL << endl;
                
                int_e_v[i] = (int_e_v[i] > 0) ? ctrl_param.int_max[i] : -ctrl_param.int_max[i];
            }
        }else
        {
            int_e_v[i] = 0;
        }
    }

    // z int
    float int_start_error = 0.2;
    if(abs(pos_error[2]) < int_start_error && uav_state.mode == "OFFBOARD")
    {
        int_e_v[2] += pos_error[2] / controller_hz;

        if(abs(int_e_v[2]) > ctrl_param.int_max[2])
        {
            // cout << YELLOW << "----> int_e_v saturation [ "<< 2 << " ]"<<" [ctrl_param.int_max]: "<<ctrl_param.int_max[2]<<" [m/s] "<< TAIL << endl;
            
            int_e_v[2] = (int_e_v[2] > 0) ? ctrl_param.int_max[2] : -ctrl_param.int_max[2];
        }
    }else
    {
        int_e_v[2] = 0;
    }

    Eigen::Vector3d u_v = u_vel + ctrl_param.Kvi* int_e_v;

	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    // F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
    // u_att是用于PX4的姿态控制输入，u_att 前三位是roll pitch yaw， 第四位为油门值[0-1]
    Eigen::Vector3d F_des;
	F_des = u_v * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g + ctrl_param.Ka * ctrl_param.quad_mass * desired_state.acc;
    
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
		// ROS_INFO("pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
		// ROS_INFO("roll too tilt");
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

void pos_controller_PID::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  PID Position Controller  <<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << GREEN << "----> In attitude control mode. " << TAIL << endl;
    cout << GREEN << "----> Debug Info: " << TAIL << endl;
    cout << GREEN << "----> pos_des : " << desired_state.pos(0) << " [ m ] " << desired_state.pos(1) << " [ m ] " << desired_state.pos(2) << " [ m ] "<< TAIL << endl;
    cout << GREEN << "----> u_attitude: " << u_att(0)*180/3.14 << " [deg] "<< u_att(1)*180/3.14 << " [deg] "<< u_att(2)*180/3.14 << " [deg] "<< TAIL << endl;
    cout << GREEN << "----> u_throttle: " << u_att(3) << " [0-1] "<< TAIL << endl;
    cout << "pos_error_mean : " << tracking_error.pos_error_mean <<" [m] "<<endl;
    cout << "vel_error_mean : " << tracking_error.vel_error_mean <<" [m/s] "<<endl;
}

// 【打印参数函数】
void pos_controller_PID::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "ctrl_param.quad_mass   : "<< ctrl_param.quad_mass<<endl;
    cout << "ctrl_param.hov_percent   : "<< ctrl_param.hov_percent<<endl;
    cout << "pxy_int_max   : "<< ctrl_param.int_max[0]<<endl;
    cout << "pz_int_max   : "<< ctrl_param.int_max[2]<<endl;

    cout << "hover_gain/Kp_xy   : "<< ctrl_param_hover.Kp(0,0) <<endl;
    cout << "hover_gain/Kp_z    : "<< ctrl_param_hover.Kp(2,2) <<endl;
    cout << "hover_gain/Kv_xy   : "<< ctrl_param_hover.Kv(0,0) <<endl;
    cout << "hover_gain/Kv_z    : "<< ctrl_param_hover.Kv(2,2) <<endl;
    cout << "hover_gain/Kvi_xy   : "<< ctrl_param_hover.Kvi(0,0) <<endl;
    cout << "hover_gain/Kvi_z    : "<< ctrl_param_hover.Kvi(2,2) <<endl;
    cout << "hover_gain/Ka_xy   : "<< ctrl_param_hover.Ka(0,0) <<endl;
    cout << "hover_gain/Ka_z    : "<< ctrl_param_hover.Ka(2,2) <<endl;
    cout << "hover_gain/tilt_angle_max    : "<< ctrl_param_hover.tilt_angle_max <<endl;

    cout << "track_gain/Kp_xy   : "<< ctrl_param_track.Kp(0,0) <<endl;
    cout << "track_gain/Kp_z    : "<< ctrl_param_track.Kp(2,2) <<endl;
    cout << "track_gain/Kv_xy   : "<< ctrl_param_track.Kv(0,0) <<endl;
    cout << "track_gain/Kv_z    : "<< ctrl_param_track.Kv(2,2) <<endl;
    cout << "track_gain/Kvi_xy   : "<< ctrl_param_track.Kvi(0,0) <<endl;
    cout << "track_gain/Kvi_z    : "<< ctrl_param_track.Kvi(2,2) <<endl;
    cout << "track_gain/Ka_xy   : "<< ctrl_param_track.Ka(0,0) <<endl;
    cout << "track_gain/Ka_z    : "<< ctrl_param_track.Ka(2,2) <<endl;
    cout << "track_gain/tilt_angle_max    : "<< ctrl_param_track.tilt_angle_max <<endl;
}
#endif