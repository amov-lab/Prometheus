#ifndef UGV_CONTROLLER_H
#define UGV_CONTROLLER_H
#include <ros/ros.h>
#include <bitset>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/UgvCommand.h>

#include "math_utils.h"
#include "Smoother.h"
#include "angles/angles.h"  // vinson: shortest_angular_distance
#include "printf_utils.h"

// 宏定义
#define NUM_POINT 2                             // 打印小数点

// 变量
int ugv_id;                                               // 无人车编号
string ugv_name;                                // 无人车名字
Eigen::Vector2f geo_fence_x,geo_fence_y;                           //Geigraphical fence 地理围栏
prometheus_msgs::UgvCommand Command_Now;      // 无人车当前执行命令
prometheus_msgs::UgvCommand Command_Last;       // 无人车上一条执行命令
prometheus_msgs::UgvState _UgvState;                             // 无人车状态
Eigen::Vector3d pos_ugv;                      // 无人车位置
Eigen::Vector3d vel_ugv;                      // 无人车速度
float yaw_ugv;                                          // 无人车偏航角
float k_p,k_yaw;                                      // 速度控制参数
float max_vel;                                          // 无人车最大速度
bool flag_printf;                                      // 是否打印
geometry_msgs::Twist cmd_vel;      // 底层速度指令   

// 订阅
ros::Subscriber command_sub;
ros::Subscriber ugv_state_sub;

// 发布
ros::Publisher cmd_pub;

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
    // 控制变量
    nh.param("k_p", k_p, 2.0f);
    nh.param("k_yaw", k_yaw, 2.0f);
    nh.param("max_vel", max_vel, 2.0f);
    // 是否打印消息
    nh.param("flag_printf", flag_printf, false);
    // 地理围栏
    nh.param("geo_fence/x_min", geo_fence_x[0], -100.0f);
    nh.param("geo_fence/x_max", geo_fence_x[1], 100.0f);
    nh.param("geo_fence/y_min", geo_fence_y[0], -100.0f);
    nh.param("geo_fence/y_max", geo_fence_y[1], 100.0f);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 初始化命令
    Command_Now.Mode                = prometheus_msgs::UgvCommand::Hold;
    Command_Now.Command_ID          = 0;
    Command_Now.linear_vel[0]       = 0;
    Command_Now.linear_vel[1]       = 0;
    Command_Now.angular_vel         = 0;

    sm = TB2::Smoother("smoother");
    only_rotate = true;

    cout << GREEN << "ugv_controller_ugv_" <<  ugv_id << " init."<< TAIL <<endl; 
    cout << "ugv_name   : "<< ugv_name <<endl;
    cout << "k_p    : "<< k_p <<"  "<<endl;
    cout << "k_yaw    : "<< k_yaw <<"  "<<endl;
    cout << "max_vel    : "<< max_vel <<"  "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
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
    else
    {
        return 0;
    }
}

#endif