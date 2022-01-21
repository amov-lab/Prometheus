#ifndef PX4_EGO_CONTROLLER_H
#define PX4_EGO_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/SwarmCommand.h>

#include "control_utils.h"
#include "uav_utils/geometry_utils.h"
#include "printf_utils.h"

using namespace std;
int swarm_num;                                  // 集群数量
string uav_name;                                // 无人机名字
int uav_id;                                     // 无人机编号

ros::Subscriber drone_state_sub;
ros::Subscriber odom_sub;
ros::Subscriber cmd_sub;

ros::Publisher setpoint_raw_local_pub, setpoint_raw_attitude_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

prometheus_msgs::SwarmCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::SwarmCommand Command_Last;                     //无人机上一条执行命令
Eigen::Vector3d Takeoff_position;                                // 起飞位置

mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

prometheus_msgs::DroneState _DroneState;                          //无人机状态量
nav_msgs::Odometry drone_odom;
Eigen::Vector3d pos_drone;                      // 无人机位置
Eigen::Vector3d vel_drone;                      // 无人机速度
Eigen::Quaterniond q_drone;                 // 无人机四元数
double yaw_drone;

Eigen::Vector3d pos_des;         
Eigen::Vector3d vel_des;         
Eigen::Vector3d acc_des;                   
double yaw_des;  
float int_start_error;
Eigen::Vector3d int_e_v;            // 积分
Eigen::Quaterniond u_q_des;   // 期望姿态角（四元数）
Eigen::Vector4d u_att;                  // 期望姿态角（rad）+期望油门（0-1）

// 控制参数
Eigen::Matrix3d Kp_hover;
Eigen::Matrix3d Kv_hover;
Eigen::Matrix3d Kvi_hover;
Eigen::Matrix3d Ka_hover;
float tilt_angle_max_hover;
Eigen::Matrix3d Kp_track;
Eigen::Matrix3d Kv_track;
Eigen::Matrix3d Kvi_track;
Eigen::Matrix3d Ka_track;
float tilt_angle_max_track;
Eigen::Matrix3d Kp;
Eigen::Matrix3d Kv;
Eigen::Matrix3d Kvi;
Eigen::Matrix3d Ka;
float tilt_angle_max;
Eigen::Vector3d g_;
float quad_mass;
float hov_percent;      // 0-1
Eigen::Vector3f int_max;

void init()
{
    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode                                = prometheus_msgs::SwarmCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Move_mode           = prometheus_msgs::SwarmCommand::XYZ_POS;
    Command_Now.position_ref[0]     = 0;
    Command_Now.position_ref[1]     = 0;
    Command_Now.position_ref[2]     = 0;
    Command_Now.velocity_ref[0]     = 0;
    Command_Now.velocity_ref[1]     = 0;
    Command_Now.velocity_ref[2]     = 0;
    Command_Now.acceleration_ref[0] = 0;
    Command_Now.acceleration_ref[1] = 0;
    Command_Now.acceleration_ref[2] = 0;
    Command_Now.yaw_ref             = 0;

    int_start_error = 2.0;
    int_e_v.setZero();
    u_att.setZero();
    g_ << 0.0, 0.0, 9.8;
}

void debug_cb(const ros::TimerEvent &e)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN << "----> Debug Info: " << TAIL << endl;
    cout << GREEN << "----> pos_drone : " << pos_drone(0) << " [ m ] " << pos_drone(1) << " [ m ] " << pos_drone(2) << " [ m ] "<< TAIL << endl;
    cout << GREEN << "----> pos_des   : " << pos_des(0)   << " [m/s] " << pos_des(1)   << " [m/s] " << pos_des(2)   << " [m/s] "<< TAIL << endl;
    cout << GREEN << "----> u_attitude: " << u_att(0)*180/3.14 << " [deg] "<< u_att(1)*180/3.14 << " [deg] "<< u_att(2)*180/3.14 << " [deg] "<< TAIL << endl;
    cout << GREEN << "----> u_throttle: " << u_att(3) << " [0-1] "<< TAIL << endl;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}

void drone_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    drone_odom = *msg;

    pos_drone(0) = drone_odom.pose.pose.position.x;
    pos_drone(1) = drone_odom.pose.pose.position.y;
    pos_drone(2) = drone_odom.pose.pose.position.z;

    vel_drone(0) = drone_odom.twist.twist.linear.x;
    vel_drone(1) = drone_odom.twist.twist.linear.y;
    vel_drone(2) = drone_odom.twist.twist.linear.z;

    q_drone.w() = msg->pose.pose.orientation.w;
    q_drone.x() = msg->pose.pose.orientation.x;
    q_drone.y() = msg->pose.pose.orientation.y;
    q_drone.z() = msg->pose.pose.orientation.z;    

    yaw_drone = uav_utils::get_yaw_from_quaternion(q_drone);
}

void cmd_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_Now = *msg;

    if(Command_Now.Mode == prometheus_msgs::SwarmCommand::Move &&
        Command_Now.Move_mode == prometheus_msgs::SwarmCommand::TRAJECTORY)
    {
        Kp = Kp_track;
        Kv = Kv_track;
        Kvi = Kvi_track;
        Ka = Ka_track;
        tilt_angle_max = tilt_angle_max_track;
    }else
    {
        Kp = Kp_hover;
        Kv = Kv_hover;
        Kvi = Kvi_hover;
        Ka = Ka_hover;
        tilt_angle_max = tilt_angle_max_hover;
    }
}

void idle()
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0x4000;
    setpoint_raw_local_pub.publish(pos_setpoint);
}

// 发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
void send_attitude_setpoint(Eigen::Vector4d& u_att)
{
    mavros_msgs::AttitudeTarget att_setpoint;
    //geometry_msgs/Quaternion

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000111;

    Eigen::Vector3d att_des;
    att_des << u_att(0), u_att(1), u_att(2);

    Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);

    att_setpoint.orientation.x = q_des.x();
    att_setpoint.orientation.y = q_des.y();
    att_setpoint.orientation.z = q_des.z();
    att_setpoint.orientation.w = q_des.w();
    att_setpoint.thrust = u_att(3);

    setpoint_raw_attitude_pub.publish(att_setpoint);
}

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
void pos_controller()
{
    // 定点的时候才积分
	if (vel_des(0) != 0.0 || vel_des(1) != 0.0 || vel_des(2) != 0.0) 
    {
		//ROS_INFO("Reset integration");
		int_e_v.setZero();
	}

    Eigen::Vector3d pos_error = pos_des - pos_drone;

    Eigen::Vector3d u_pos = Kp * pos_error;

    Eigen::Vector3d vel_error  = u_pos + vel_des - vel_drone;

    Eigen::Vector3d u_vel = Kv * vel_error;  

    Eigen::Vector3d u_int = Kvi* int_e_v;

    for (int i=0; i<3; i++)
    {
        // 只有在pos_error比较小时，才会启动积分
        if(abs(pos_error[i]) < int_start_error)
        {
            int_e_v[i] += pos_error[i] * 0.01;

            if(abs(int_e_v[i]) > int_max[i])
            {
                cout << YELLOW << "----> int_e_v saturation [ "<< i << " ]"<<" [int_max]: "<<int_max[i]<<" [m/s] "<< TAIL << endl;
                
                int_e_v[i] = (int_e_v[i] > 0) ? int_max[i] : -int_max[i];
            }
        }else
        {
            int_e_v[i] = 0;
        }

        // If not in OFFBOARD mode, set all intergral to zero.
        if(_DroneState.mode != "OFFBOARD")
        {
            int_e_v[i] = 0;
        }
    }

    Eigen::Vector3d u_v = u_vel + u_int;

	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    Eigen::Vector3d F_des;
	F_des = u_v * quad_mass + quad_mass * g_ + Ka * quad_mass * acc_des;
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * quad_mass * g_(2))
	{
		ROS_INFO("thrust too low");
		F_des = F_des / F_des(2) * (0.5 * quad_mass * g_(2));
	}
	else if (F_des(2) > 2 * quad_mass * g_(2))
	{
		ROS_INFO("thrust too high");
		F_des = F_des / F_des(2) * (2 * quad_mass * g_(2));
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(uav_utils::toRad(tilt_angle_max)))
	{
		// ROS_INFO("pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(uav_utils::toRad(tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(uav_utils::toRad(tilt_angle_max)))
	{
		// ROS_INFO("roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(uav_utils::toRad(tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = uav_utils::rotz(yaw_drone);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1)  = std::atan2( fx, fz);
    u_att(2)  = yaw_des;

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = q_drone.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的推力合力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = quad_mass * g_(2) / hov_percent;

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
}




#endif
