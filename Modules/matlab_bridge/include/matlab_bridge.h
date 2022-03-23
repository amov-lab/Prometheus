//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "geometry_utils.h"
#include "printf_utils.h"

using namespace std;

#define MATLAB_TIMEOUT 0.2
#define UAV_STATE_TIMEOUT 0.1
#define RETURN_INIT_POS_TIMEOUT 60
#define LAND_TIMEOUT 300

enum MATLAB_CMD_X
{
    CHECK = 1,      // 起飞前检查
    TAKEOFF = 2,
    LAND = 3,
    HOLD = 4,
    MATLAB_CMD = 5  // 指令控制
};

enum MATLAB_CMD_Y
{
    POS_CTRL_MODE = 1,  // 位置点控制
    VEL_CTRL_MODE = 2,  // 速度控制
    ACC_CTRL_MODE = 3,  // 加速度控制
    ATT_CTRL_MODE = 4   // 姿态控制
};

enum MATLAB_RESULT_X
{
    REJECT = 1,
    SUCCESS = 2
};

int uav_id;
int matlab_control_mode;
bool ready_for_matlab_check;
bool ready_for_matlab_cmd;
geometry_msgs::Pose matlab_cmd;
ros::Time last_matlab_cmd_time;
geometry_msgs::Point matlab_setting_cmd;
ros::Time last_matlab_setting_cmd_time;
ros::Time get_uav_state_stamp;
geometry_msgs::Point matlab_setting_result;
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;

bool cmd_timeout{false};

ros::Publisher matlab_setting_result_pub;
ros::Publisher uav_command_pub;

struct Ctrl_Param_Matlab
{
    float quad_mass;
    float tilt_angle_max;
    float hov_percent;
    Eigen::Vector3d g;
};

Ctrl_Param_Matlab ctrl_param;

Eigen::Vector4d acc_cmd_to_att_cmd(Eigen::Vector3d & acc_cmd, double yaw_cmd)
{
    Eigen::Vector4d att_cmd;
	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    // F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
    // att_cmd是用于PX4的姿态控制输入，att_cmd 前三位是roll pitch yaw， 第四位为油门值[0-1]
    Eigen::Vector3d F_des;
	F_des = acc_cmd * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g;
    
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
    double current_yaw = (double)uav_state.attitude[2];
    Eigen::Matrix3d wRc = geometry_utils::rotz(current_yaw);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    att_cmd(0)  = std::atan2(-fy, fz);
    att_cmd(1)  = std::atan2( fx, fz);
    att_cmd(2)  = yaw_cmd;

    Eigen::Quaterniond uav_quat;
    uav_quat = Eigen::Quaterniond(uav_state.attitude_q.w, uav_state.attitude_q.x, uav_state.attitude_q.y, uav_state.attitude_q.z);

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = uav_quat.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的电机推力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = ctrl_param.quad_mass * ctrl_param.g(2) / ctrl_param.hov_percent;

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    att_cmd(3) = u1 / full_thrust;

    if(att_cmd(3) < 0.1)
    {
        att_cmd(3) = 0.1;
        ROS_INFO("throttle too low");
    }

    if(att_cmd(3) > 1.0)
    {
        att_cmd(3) = 1.0;
        ROS_INFO("throttle too high");
    }

    return att_cmd;
}

bool check_for_uav_state()
{
    if(!uav_state.connected)
    {
        cout << RED  << "check_for_uav_state: not connected!" << TAIL<<endl;
        return false;
    }

    if(!uav_state.armed)
    {
        cout << RED << "check_for_uav_state: Disarm the drone first!"<< TAIL << endl;
        return false;
    }

    if(uav_state.mode != "OFFBOARD")
    {
        cout << RED  << "check_for_uav_state: not in offboard mode!" << TAIL<<endl;
        return false;
    }

    if(!uav_state.odom_valid)
    {
        cout << RED  << "check_for_uav_state: odom invalid!" << TAIL<<endl;
        return false;
    }

    return true;
}


void matlab_safety_check(const ros::TimerEvent &e)
{
    if(!ready_for_matlab_cmd)
    {
        return;
    }

    ros::Time time_now = ros::Time::now();
    // 接收uav_state超时，降落
    if((time_now - get_uav_state_stamp).toSec() > UAV_STATE_TIMEOUT)
    {
        cout << RED  << "check_for_uav_state: uav state timeout, land!" << TAIL<<endl;
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
        uav_command_pub.publish(uav_command);
        return;
    }

    bool uav_ready = check_for_uav_state();

    if(!uav_ready)
    {
        cout << RED  << "uav_state error, LAND now!" << TAIL<<endl;
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
        uav_command_pub.publish(uav_command);
        return;
    }

    double delta_time_matlab_cmd = (time_now - last_matlab_cmd_time).toSec();

    // 接收指令超时（网络状态较差会导致），原地悬停等待
    if(delta_time_matlab_cmd > MATLAB_TIMEOUT)
    {
        if(!cmd_timeout)
        {
            cout << RED  << "MATLAB_TIMEOUT!" << TAIL<<endl;
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_command_pub.publish(uav_command);
        }else
        {
            // 接收指令超时，降落
            if(delta_time_matlab_cmd > LAND_TIMEOUT)
            {
                cout << RED  << "MATLAB_TIMEOUT, LAND now!" << TAIL<<endl;
                uav_command.header.stamp = ros::Time::now();
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                uav_command_pub.publish(uav_command);
            }
            // 接收指令超时，返回起始点
            else if(delta_time_matlab_cmd > RETURN_INIT_POS_TIMEOUT)
            {
                cout << RED  << "MATLAB_TIMEOUT，RETURN Init Pos!" << TAIL<<endl;
                uav_command.header.stamp = ros::Time::now();
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command_pub.publish(uav_command);
            }
        }
        
        cmd_timeout = true;
    }
}

void printf_msgs(const ros::TimerEvent &e)
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

    if(matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        if(matlab_control_mode == MATLAB_CMD_Y::POS_CTRL_MODE)
        {
            cout << GREEN  << "Command: [ Move in XYZ_POS ] " << TAIL<<endl;
            cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
            cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
        }else if(matlab_control_mode == MATLAB_CMD_Y::VEL_CTRL_MODE)
        {
            cout << GREEN  << "Command: [ Move in XYZ_VEL ] " << TAIL<<endl;
            cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
            cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
        }else if(matlab_control_mode == MATLAB_CMD_Y::ATT_CTRL_MODE)
        {
            cout << GREEN  << "Command: [ Move in XYZ_ATT ] " << TAIL<<endl;
            cout << GREEN  << "Att_ref [X Y Z] : " << uav_command.att_ref[0] * 180/M_PI<< " [deg] "<< uav_command.att_ref[1]* 180/M_PI<<" [deg] "<< uav_command.att_ref[2]* 180/M_PI<<" [deg] "<< TAIL<<endl;
            cout << GREEN  << "Thrust_ref[0-1] : " << uav_command.att_ref[3] << TAIL<<endl;
        }
    }
}