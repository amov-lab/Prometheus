#ifndef UGV_CONTROLLER_H
#define UGV_CONTROLLER_H
#include <ros/ros.h>
#include <bitset>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <geographic_msgs/GeoPointStamped.h>

#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/UgvCommand.h>

#include "message_utils.h"
#include "math_utils.h"
#include "Smoother.h"

#include "angles/angles.h"  // vinson: shortest_angular_distance
#include "Smoother.h"
#include "printf_utils.h"
#include "nav_msgs/Odometry.h"



// 宏定义
#define NUM_POINT 2                             // 打印小数点

// 变量
int ugv_id;
string ugv_name;                                // 无人机名字
bool sim_mode;
Eigen::Vector2f geo_fence_x,geo_fence_y; //Geigraphical fence 地理围栏
prometheus_msgs::UgvCommand Command_Now;      // 无人机当前执行命令
prometheus_msgs::UgvCommand Command_Last;     // 无人机上一条执行命令
prometheus_msgs::UgvState _UgvState;        // 无人机状态
Eigen::Vector3d pos_ugv;                      // 无人机位置
Eigen::Vector3d vel_ugv;                      // 无人机速度
float yaw_ugv;
float k_p,k_yaw;                                      // 速度控制参数
bool flag_printf;                               // 是否打印
geometry_msgs::Twist cmd_vel;
Eigen::Vector3d state_sp;

nav_msgs::Odometry _odom;

// 订阅
ros::Subscriber command_sub;
ros::Subscriber ugv_state_sub;
ros::Subscriber odom_sub;

// 发布
ros::Publisher turtlebot_cmd_pub;
ros::Publisher setpoint_raw_local_pub, ekf_origin_pub,vel_body_pub;
ros::Publisher message_pub;

// 服务
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

/// @brief 速度平滑器 
/// @details 无视了小车加速度，简单实现了速度的平滑控制，使得小车的速度不会变化太大
TB2::Smoother sm;

double dist,direct,yaw_error;
bool only_rotate;
bool first_arrive;  // 在距离上到达并停下之前都为false
bool stop_flag;

void init(ros::NodeHandle &nh)
{
    nh.param("ugv_id", ugv_id, 0);
    nh.param("sim_mode", sim_mode, true);
    // 控制变量
    nh.param("k_p", k_p, 1.0f);
    nh.param("k_yaw", k_yaw, 2.0f);
    // 是否打印消息
    nh.param("flag_printf", flag_printf, false);
    // 地理围栏
    nh.param("geo_fence/x_min", geo_fence_x[0], -100.0f);
    nh.param("geo_fence/x_max", geo_fence_x[1], 100.0f);
    nh.param("geo_fence/y_min", geo_fence_y[0], -100.0f);
    nh.param("geo_fence/y_max", geo_fence_y[1], 100.0f);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 初始化命令
    Command_Now.Mode                = prometheus_msgs::UgvCommand::Start;
    Command_Now.Command_ID          = 0;
    Command_Now.linear_vel[0]       = 0;
    Command_Now.linear_vel[1]       = 0;
    Command_Now.angular_vel         = 0;

    sm = TB2::Smoother("smoother");
    only_rotate = true;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void ugv_command_cb(const prometheus_msgs::UgvCommand::ConstPtr& msg)
{
    Command_Now = *msg;
    only_rotate = true; // vinson: should be set for initializing.
    first_arrive = true;
    stop_flag = false;
}

void ugv_state_cb(const prometheus_msgs::UgvState::ConstPtr& msg)
{
    _UgvState = *msg;

    pos_ugv  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_ugv  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    yaw_ugv = _UgvState.attitude[2];
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    _odom = *msg;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> ugv controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "ugv_name   : "<< ugv_name <<endl;
    cout << "k_p    : "<< k_p <<"  "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
}

void printf_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Ugv Controller  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
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

    cout << "UAV_name : " <<  ugv_name << endl;
    cout << "UAV_pos [X Y Z] : " << pos_ugv[0] << " [ m ] "<< pos_ugv[1]<<" [ m ] "<<pos_ugv[2]<<" [ m ] "<<endl;
    cout << "UAV_vel [X Y Z] : " << vel_ugv[0] << " [ m/s ] "<< vel_ugv[1]<<" [ m/s ] "<<vel_ugv[2]<<" [ m/s ] "<<endl;
}

int check_failsafe()
{
    if (_UgvState.position[0] < geo_fence_x[0] || _UgvState.position[0] > geo_fence_x[1] ||
        _UgvState.position[1] < geo_fence_y[0] || _UgvState.position[1] > geo_fence_y[1])
    {
        cout << RED << "Out of the geo fence, stop!" << TAIL <<endl; 
        return 1;
    }
    else{
        return 0;
    }
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

#endif