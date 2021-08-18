#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H

#include <ros/ros.h>
#include <bitset>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "formation_utils.h"
#include "message_utils.h"
#include "uav_utils/geometry_utils.h"
#include "math_utils.h"
#include "printf_utils.h"

using namespace std;

// 宏定义
#define NODE_NAME "swarm_controller"            // 节点名字
#define NUM_POINT 2                             // 打印小数点
#define MAX_UAV_NUM 50

// 变量
int swarm_num_uav;                                  // 集群数量
string uav_name;                                // 无人机名字
int uav_id;                                     // 无人机编号
int num_neighbour = 2;                          // 邻居数量,目前默认为2
int neighbour_id1,neighbour_id2;                // 邻居ID
string neighbour_name1,neighbour_name2;         // 邻居名字
int collision_flag;
int controller_flag;
int controller_hz;
string msg_name;
Eigen::Vector2f geo_fence_x,geo_fence_y,geo_fence_z; //Geigraphical fence 地理围栏
prometheus_msgs::SwarmCommand Command_Now;      // 无人机当前执行命令
prometheus_msgs::SwarmCommand Command_Last;     // 无人机上一条执行命令


float Takeoff_height;                           // 默认起飞高度
Eigen::Vector3d Takeoff_position;               // 起飞位置
float Disarm_height;                            // 自动上锁高度
float Land_speed;                               // 降落速度
bool flag_printf;                               // 是否打印
int Land_mode;                                  //降落模式
Eigen::Vector3f gazebo_offset;                  // 偏移量
prometheus_msgs::Message message;               // 待打印消息

// 订阅
ros::Subscriber command_sub;
ros::Subscriber drone_state_sub;
ros::Subscriber position_target_sub;
ros::Subscriber nei_state_sub[MAX_UAV_NUM+1];

// 发布
ros::Publisher setpoint_raw_local_pub;
ros::Publisher setpoint_raw_attitude_pub;
ros::Publisher message_pub;

// 服务
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

// 邻居状态量
prometheus_msgs::DroneState state_nei[MAX_UAV_NUM+1];
Eigen::Vector3d pos_nei[MAX_UAV_NUM+1];                     // 邻居位置
Eigen::Vector3d vel_nei[MAX_UAV_NUM+1];                     // 邻居速度
Eigen::Vector3d dv;
float R = 4.0;
float r = 0.5;
// 无人机状态量
Eigen::Vector3d pos_drone;                      // 无人机位置
Eigen::Vector3d vel_drone;                      // 无人机速度
Eigen::Quaterniond q_drone;                 // 无人机四元数
double yaw_drone;

// 目标设定值
prometheus_msgs::DroneState _DroneState;        // 无人机状态
Eigen::Vector3d pos_des(0,0,0);         
Eigen::Vector3d vel_des(0,0,0);         
Eigen::Vector3d acc_des(0,0,0);    
Eigen::Vector3d throttle_sp(0,0,0);                      
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

// 编队控制相关
Eigen::MatrixXf formation_separation;           // 阵型偏移量
float k_p;                                      // 速度控制参数
float k_aij;                                    // 速度控制参数
float k_gamma;                                  // 速度控制参数
float yita;                                     // 速度控制参数

void init(ros::NodeHandle &nh)
{
    // 集群数量
    nh.param<int>("swarm_num_uav", swarm_num_uav, 1);
    // 无人机编号 1号无人机则为1
    nh.param<int>("uav_id", uav_id, 0);
    uav_name = "/uav" + std::to_string(uav_id);
    // 控制器标志位: 0代表姿态（使用本程序的位置环控制算法），1代表位置、速度（使用PX4的位置环控制算法）
    nh.param<int>("controller_flag", controller_flag, 1);
    nh.param<int>("controller_hz", controller_hz, 50);
    nh.param<int>("collision_flag", collision_flag, 1);
    
    // 起飞高度,上锁高度,降落速度,降落模式
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.2);
    nh.param<float>("Land_speed", Land_speed, 0.2);
    nh.param<int>("Land_mode", Land_mode, 0);
    // 是否打印消息
    nh.param<bool>("flag_printf", flag_printf, false);
    // 地理围栏
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);
    // 如果是使用的ekf2_gps则需要设置，如果使用的是ekf2_vision则不需要
    nh.param<float>("gazebo_offset_x", gazebo_offset[0], 0);
    nh.param<float>("gazebo_offset_y", gazebo_offset[1], 0);
    nh.param<float>("gazebo_offset_z", gazebo_offset[2], 0);

    nh.param<float>("quad_mass" , quad_mass, 1.5f);
    nh.param<float>("hov_percent" , hov_percent, 0.47f);
    nh.param<float>("Limit/pxy_int_max"  , int_max[0], 5.0);
    nh.param<float>("Limit/pxy_int_max"  , int_max[1], 5.0);
    nh.param<float>("Limit/pz_int_max"   , int_max[2], 10.0);
    nh.param<double>("hover_gain/Kp_xy", Kp_hover(0,0), 2.0f);
    nh.param<double>("hover_gain/Kp_xy", Kp_hover(1,1), 2.0f);
    nh.param<double>("hover_gain/Kp_z" , Kp_hover(2,2), 2.0f);
    nh.param<double>("hover_gain/Kv_xy", Kv_hover(0,0), 2.0f);
    nh.param<double>("hover_gain/Kv_xy", Kv_hover(1,1), 2.0f);
    nh.param<double>("hover_gain/Kv_z" , Kv_hover(2,2), 2.0f);
    nh.param<double>("hover_gain/Kvi_xy", Kvi_hover(0,0), 0.3f);
    nh.param<double>("hover_gain/Kvi_xy", Kvi_hover(1,1), 0.3f);
    nh.param<double>("hover_gain/Kvi_z" , Kvi_hover(2,2), 0.3f);
    nh.param<double>("hover_gain/Ka_xy", Ka_hover(0,0), 1.0f);
    nh.param<double>("hover_gain/Ka_xy", Ka_hover(1,1), 1.0f);
    nh.param<double>("hover_gain/Ka_z" , Ka_hover(2,2), 1.0f);
    nh.param<float>("hover_gain/tilt_angle_max" , tilt_angle_max_hover, 10.0f);

    nh.param<double>("track_gain/Kp_xy", Kp_track(0,0), 3.0f);
    nh.param<double>("track_gain/Kp_xy", Kp_track(1,1), 3.0f);
    nh.param<double>("track_gain/Kp_z" , Kp_track(2,2), 3.0f);
    nh.param<double>("track_gain/Kv_xy", Kv_track(0,0), 3.0f);
    nh.param<double>("track_gain/Kv_xy", Kv_track(1,1), 3.0f);
    nh.param<double>("track_gain/Kv_z" , Kv_track(2,2), 3.0f);
    nh.param<double>("track_gain/Kvi_xy", Kvi_track(0,0), 0.1f);
    nh.param<double>("track_gain/Kvi_xy", Kvi_track(1,1), 0.1f);
    nh.param<double>("track_gain/Kvi_z" , Kvi_track(2,2), 0.1f);
    nh.param<double>("track_gain/Ka_xy", Ka_track(0,0), 1.0f);
    nh.param<double>("track_gain/Ka_xy", Ka_track(1,1), 1.0f);
    nh.param<double>("track_gain/Ka_z" , Ka_track(2,2), 1.0f);
    nh.param<float>("track_gain/tilt_angle_max" , tilt_angle_max_track, 20.0f);
    
    // 编队控制参数
    nh.param<float>("k_p", k_p, 1.2);
    nh.param<float>("k_aij", k_aij, 0.1);
    nh.param<float>("k_gamma", k_gamma, 1.2);

    msg_name = uav_name + "/control";
    // 初始化命令
    Command_Now.Mode                = prometheus_msgs::SwarmCommand::Idle;
    Command_Now.Command_ID          = 0;
    Command_Now.position_ref[0]     = 0;
    Command_Now.position_ref[1]     = 0;
    Command_Now.position_ref[2]     = 0;
    Command_Now.yaw_ref             = 0;
    
    // 初始化阵型偏移量
    formation_separation = Eigen::MatrixXf::Zero(swarm_num_uav,4); 

    int_start_error = 2.0;
    int_e_v.setZero();
    u_att.setZero();
    g_ << 0.0, 0.0, 9.8;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    // CommandID必须递增才会被记录
    Command_Now = *msg;
    
    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == prometheus_msgs::SwarmCommand::Disarm)
    {
        Command_Now = Command_Last;
    }

    if(Command_Now.Mode == prometheus_msgs::SwarmCommand::Position_Control ||
        Command_Now.Mode == prometheus_msgs::SwarmCommand::Velocity_Control ||
        Command_Now.Mode == prometheus_msgs::SwarmCommand::Accel_Control )
    {
        if (swarm_num_uav == 1)
        {
            // swarm_num_uav 为1时,即无人机无法变换阵型,并只能接收位置控制指令
            Command_Now.Mode = prometheus_msgs::SwarmCommand::Position_Control;
            formation_separation << 0,0,0,0;
        }else if(swarm_num_uav == 4 || swarm_num_uav == 8 || swarm_num_uav == 40)
        {
            formation_separation = formation_utils::get_formation_separation(Command_Now.swarm_shape, Command_Now.swarm_size, swarm_num_uav);
        }else
        {
            // 未指定阵型,如若想6机按照8机编队飞行,则设置swarm_num_uav为8即可
            Command_Now.Mode = prometheus_msgs::SwarmCommand::Position_Control;
            formation_separation = Eigen::MatrixXf::Zero(swarm_num_uav,4); 
            pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_name, "Wrong swarm_num_uav");
        }
    }
    // 切换参数
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

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    pos_drone  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_drone  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    q_drone.w() = msg->attitude_q.w;
    q_drone.x() = msg->attitude_q.x;
    q_drone.y() = msg->attitude_q.y;
    q_drone.z() = msg->attitude_q.z;    

    yaw_drone = uav_utils::get_yaw_from_quaternion(q_drone);
}

void nei_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int nei_id)
{
    state_nei[nei_id] = *msg;

    pos_nei[nei_id]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_nei[nei_id]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}

int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        cout << RED << "----> Out of the geo fence, the drone is landing..."<< TAIL << endl;
        return 1;
    }
    else{
        return 0;
    }
}

void idle()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    pos_setpoint.type_mask = 0x4000;
    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送位置期望值至飞控（输入：期望xyz,期望yaw）
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
void send_vel_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011;   // 100 111 000 011  vx vy vz z + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
    
    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

void send_pos_vel_xy_pos_z_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 2. position setpoint + velocity setpoint (velocity used as feedforward)
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void send_pos_vel_xyz_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 2. position setpoint + velocity setpoint (velocity used as feedforward)
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送加速度期望值至飞控（输入：期望axayaz,期望yaw）
void send_acc_xyz_setpoint(const Eigen::Vector3d& accel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100000111111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.acceleration_or_force.x = accel_sp[0];
    pos_setpoint.acceleration_or_force.y = accel_sp[1];
    pos_setpoint.acceleration_or_force.z = accel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Acc_target [X Y Z] : " << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
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
// roll pitch yaw 油门（0-1）
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


// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系，yaw_angle是当前偏航角[rad]
void rotation_yaw(float yaw_angle, float body_frame[2], float enu_frame[2])
{
    enu_frame[0] = body_frame[0] * cos(yaw_angle) - body_frame[1] * sin(yaw_angle);
    enu_frame[1] = body_frame[0] * sin(yaw_angle) + body_frame[1] * cos(yaw_angle);
}


void debug_cb(const ros::TimerEvent &e)
{
    if(!flag_printf)
    {
        return;
    }

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Swarm Controller  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN  << "swarm_num_uav : " <<  swarm_num_uav << TAIL << endl;
    cout << GREEN  << "UAV_id : " <<  uav_id << "   UAV_name : " <<  uav_name << TAIL << endl;
    cout << GREEN  << "UAV_pos [X Y Z] : " << pos_drone[0] << " [ m ] "<< pos_drone[1]<<" [ m ] "<<pos_drone[2]<<" [ m ] "<< TAIL <<endl;
    cout << GREEN  << "UAV_vel [X Y Z] : " << vel_drone[0] << " [ m/s ] "<< vel_drone[1]<<" [ m/s ] "<<vel_drone[2]<<" [ m/s ] "<< TAIL <<endl;

    for(int i = 1; i <= swarm_num_uav; i++) 
    {
        if(i == uav_id)
        {
            continue;
        }

        cout << GREEN  << "nei" <<i<< "_pos [X Y Z] : " << pos_nei[i][0] << " [ m ] "<< pos_nei[i][1]<<" [ m ] "<<pos_nei[i][2]<<" [ m ] "<< TAIL <<endl;
        cout << GREEN  << "nei" <<i<< "_vel [X Y Z] : " << vel_nei[i][0] << " [ m/s ] "<< vel_nei[i][1]<<" [ m/s ] "<<vel_nei[i][2]<<" [ m/s ] "<< TAIL <<endl;
    }



    if(controller_flag == 0)
    {
        cout << GREEN << "----> In attitude control mode. " << TAIL << endl;
        cout << GREEN << "----> Debug Info: " << TAIL << endl;
        cout << GREEN << "----> pos_drone : " << pos_drone(0) << " [ m ] " << pos_drone(1) << " [ m ] " << pos_drone(2) << " [ m ] "<< TAIL << endl;
        cout << GREEN << "----> pos_des   : " << pos_des(0)   << " [m/s] " << pos_des(1)   << " [m/s] " << pos_des(2)   << " [m/s] "<< TAIL << endl;
        cout << GREEN << "----> u_attitude: " << u_att(0)*180/3.14 << " [deg] "<< u_att(1)*180/3.14 << " [deg] "<< u_att(2)*180/3.14 << " [deg] "<< TAIL << endl;
        cout << GREEN << "----> u_throttle: " << u_att(3) << " [0-1] "<< TAIL << endl;
    }else
    {
        cout << GREEN << "----> In pos/vel control mode. " << TAIL << endl;
        cout << GREEN << "----> Debug Info: " << TAIL << endl;
        cout << GREEN << "----> dv : " << dv(0) << " [ m ] " << dv(1) << " [ m ] " << dv(2) << " [ m ] "<< TAIL << endl;
    }
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> swarm controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "uav_name   : "<< uav_name <<endl;
    cout << "neighbour_name1   : "<< neighbour_name1 <<endl;
    cout << "neighbour_name2   : "<< neighbour_name2 <<endl;
    cout << "k_p    : "<< k_p <<"  "<<endl;
    cout << "k_aij       : "<< k_aij <<"  "<<endl;
    cout << "k_gamma       : "<< k_gamma <<"  "<<endl;
    cout << "Takeoff_height   : "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height    : "<< Disarm_height <<" [m] "<<endl;
    cout << "Land_speed       : "<< Land_speed <<" [m/s] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
}
#endif