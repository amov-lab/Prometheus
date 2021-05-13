#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H
#include <ros/ros.h>
#include <bitset>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <prometheus_msgs/DroneState.h>

#include "formation_utils.h"
#include "message_utils.h"
#include "math_utils.h"

// 宏定义
#define NODE_NAME "swarm_controller"            // 节点名字
#define NUM_POINT 2                             // 打印小数点

// 变量
int swarm_num;                                  // 集群数量
string uav_name;                                // 无人机名字
int uav_id;                                     // 无人机编号
int num_neighbour = 2;                          // 邻居数量,目前默认为2
int neighbour_id1,neighbour_id2;                // 邻居ID
string neighbour_name1,neighbour_name2;         // 邻居名字
string msg_name;
Eigen::Vector2f geo_fence_x,geo_fence_y,geo_fence_z; //Geigraphical fence 地理围栏
prometheus_msgs::SwarmCommand Command_Now;      // 无人机当前执行命令
prometheus_msgs::SwarmCommand Command_Last;     // 无人机上一条执行命令
prometheus_msgs::DroneState _DroneState;        // 无人机状态
Eigen::Vector3d pos_drone;                      // 无人机位置
Eigen::Vector3d vel_drone;                      // 无人机速度
Eigen::Vector3d pos_nei[2];                     // 邻居位置
Eigen::Vector3d vel_nei[2];                     // 邻居速度
float Takeoff_height;                           // 默认起飞高度
Eigen::Vector3d Takeoff_position;               // 起飞位置
float Disarm_height;                            // 自动上锁高度
float Land_speed;                               // 降落速度
Eigen::MatrixXf formation_separation;           // 阵型偏移量
float k_p;                                      // 速度控制参数
float k_aij;                                    // 速度控制参数
float k_gamma;                                  // 速度控制参数
float yita;                                     // 速度控制参数
bool flag_printf;                               // 是否打印
Eigen::Vector3f gazebo_offset;                  // 偏移量
Eigen::Vector3d state_sp(0,0,0);                // 辅助变量
Eigen::Vector3d state_sp_extra(0,0,0);          // 辅助变量
float yaw_sp;                                   // 辅助变量
Eigen::Vector3d accel_sp;                       // 辅助变量
Eigen::Vector3d throttle_sp;                    // 辅助变量
prometheus_msgs::Message message;               // 待打印消息

// 订阅
ros::Subscriber command_sub;
ros::Subscriber drone_state_sub;
ros::Subscriber position_target_sub;
ros::Subscriber nei1_state_sub;
ros::Subscriber nei2_state_sub;

// 发布
ros::Publisher setpoint_raw_local_pub;
ros::Publisher message_pub;

// 服务
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

void init()
{
    // 初始化命令
    Command_Now.Mode                = prometheus_msgs::SwarmCommand::Idle;
    Command_Now.Command_ID          = 0;
    Command_Now.position_ref[0]     = 0;
    Command_Now.position_ref[1]     = 0;
    Command_Now.position_ref[2]     = 0;
    Command_Now.yaw_ref             = 0;
    
    // 初始化阵型偏移量
    formation_separation = Eigen::MatrixXf::Zero(swarm_num,4); 

    // cout << "swarm_num   : "<< swarm_num <<endl;
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
        if (swarm_num == 1)
        {
            // swarm_num 为1时,即无人机无法变换阵型,并只能接收位置控制指令
            Command_Now.Mode = prometheus_msgs::SwarmCommand::Position_Control;
            formation_separation << 0,0,0,0;
        }else if(swarm_num == 8 || swarm_num == 40)
        {
            formation_separation = formation_utils::get_formation_separation(Command_Now.swarm_shape, Command_Now.swarm_size, swarm_num);
        }else
        {
            // 未指定阵型,如若想6机按照8机编队飞行,则设置swarm_num为8即可
            Command_Now.Mode = prometheus_msgs::SwarmCommand::Position_Control;
            formation_separation = Eigen::MatrixXf::Zero(swarm_num,4); 
            pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_name, "Wrong swarm_num");
        }
    }
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    pos_drone  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_drone  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}

void nei_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int nei_id)
{
    pos_nei[nei_id]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_nei[nei_id]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
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

void printf_state()
{
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

    cout << "UAV_id : " <<  uav_id << "   UAV_name : " <<  uav_name << endl;
    cout << "neighbour_id1 : " <<  neighbour_id1 << "   neighbour_name1 : " <<  neighbour_name1 << endl;
    cout << "neighbour_id2 : " <<  neighbour_id2 << "   neighbour_name2 : " <<  neighbour_name2 << endl;
    cout << "UAV_pos [X Y Z] : " << pos_drone[0] << " [ m ] "<< pos_drone[1]<<" [ m ] "<<pos_drone[2]<<" [ m ] "<<endl;
    cout << "UAV_vel [X Y Z] : " << vel_drone[0] << " [ m/s ] "<< vel_drone[1]<<" [ m/s ] "<<vel_drone[2]<<" [ m/s ] "<<endl;
    cout << "neighbour_pos [X Y Z] : " << pos_nei[0][0] << " [ m ] "<< pos_nei[0][1]<<" [ m ] "<<pos_nei[0][2]<<" [ m ] "<<endl;
    cout << "neighbour_vel [X Y Z] : " << vel_nei[0][0] << " [ m/s ] "<< vel_nei[0][1]<<" [ m/s ] "<<vel_nei[0][2]<<" [ m/s ] "<<endl;
    cout << "neighbour_pos [X Y Z] : " << pos_nei[1][0] << " [ m ] "<< pos_nei[1][1]<<" [ m ] "<<pos_nei[1][2]<<" [ m ] "<<endl;
    cout << "neighbour_vel [X Y Z] : " << vel_nei[1][0] << " [ m/s ] "<< vel_nei[1][1]<<" [ m/s ] "<<vel_nei[1][2]<<" [ m/s ] "<<endl;
}

int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_name, "Out of the geo fence, the drone is landing...");
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

void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d& state_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011;   // 100 111 000 011  vx vy vz z + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = state_sp[0];
    pos_setpoint.velocity.y = state_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = state_sp[2];

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

// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系，yaw_angle是当前偏航角[rad]
void rotation_yaw(float yaw_angle, float body_frame[2], float enu_frame[2])
{
    enu_frame[0] = body_frame[0] * cos(yaw_angle) - body_frame[1] * sin(yaw_angle);
    enu_frame[1] = body_frame[0] * sin(yaw_angle) + body_frame[1] * cos(yaw_angle);
}

#define NUM_MOTOR 4
#define MOTOR_P1 -0.00069
#define MOTOR_P2 0.01271
#define MOTOR_P3 -0.07948
#define MOTOR_P4 0.3052
#define MOTOR_P5 0.008775
#define thrust_max_single_motor 6.0

Eigen::Vector3d accelToThrottle(const Eigen::Vector3d& accel_sp, float mass, float tilt_max)
{
    Eigen::Vector3d thrust_sp;

    //除以电机个数得到单个电机的期望推力
    thrust_sp = mass * accel_sp / NUM_MOTOR;

    // 推力限幅，根据最大倾斜角及最大油门
    float thrust_max_XY_tilt = fabs(thrust_sp[2]) * tanf(tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(thrust_max_single_motor * thrust_max_single_motor - pow(thrust_sp[2],2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    if ((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)) > pow(thrust_max_XY,2)) 
    {
        float mag = sqrtf((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)));
        thrust_sp[0] = thrust_sp[0] / mag * thrust_max_XY;
        thrust_sp[1] = thrust_sp[1] / mag * thrust_max_XY;
    }

    Eigen::Vector3d throttle_sp;

    //电机模型，可通过辨识得到，推力-油门曲线
    for (int i=0; i<3; i++)
    {
        throttle_sp[i] = MOTOR_P1 * pow(thrust_sp[i],4) + MOTOR_P2 * pow(thrust_sp[i],3) + MOTOR_P3 * pow(thrust_sp[i],2) + MOTOR_P4 * thrust_sp[i] + MOTOR_P5;
        // PX4内部默认假设 0.5油门为悬停推力 ， 在无人机重量为1kg时，直接除20得到0.5
        // throttle_sp[i] = thrust_sp[i]/20;
    }

    return throttle_sp;   
}

#endif