#ifndef UGV_CONTROLLER_H
#define UGV_CONTROLLER_H
#include <ros/ros.h>
#include <bitset>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <prometheus_msgs/UGVState.h>
#include <prometheus_msgs/MultiUGVState.h>
#include <prometheus_msgs/UGVCommand.h>

#include "math_utils.h"
#include "Smoother.h"
#include "angles/angles.h"  // vinson: shortest_angular_distance
#include "printf_utils.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>


// 宏定义
#define NUM_POINT 2                             // 打印小数点
using namespace std;

class UGV_controller
{
public:

    UGV_controller(ros::NodeHandle& nh);
    void mainloop();



    // 变量
    int ugv_id,swarm_num_ugv,id;                                               // 无人车编号
    string ugv_name;                                // 无人车名字
    Eigen::Vector2f geo_fence_x,geo_fence_y;                           //Geigraphical fence 地理围栏
    prometheus_msgs::UGVCommand Command_Now;      // 无人车当前执行命令
    prometheus_msgs::UGVCommand Command_Last;       // 无人车上一条执行命令
    prometheus_msgs::UGVState _UgvState;                             // 无人车状态
    Eigen::Vector3d pos_ugv;                      // 无人车位置
    Eigen::Vector3d vel_ugv;                      // 无人车速度

    Eigen::Vector3d ugv_circle_pos_;



    float yaw_ugv;                                          // 无人车偏航角
    float k_p,k_p_path,k_yaw,k_aoivd,k_i;                                      // 速度控制参数
    float max_vel;               // 无人车最大速度
    Eigen::Vector2f integral;              //
    float d_t{0.01};
    bool flag_printf;                                      // 是否打印
    float error_yaw;
    geometry_msgs::Twist cmd_vel;      // 底层速度指令   
    float circle_radius;
    float linear_vel;
    Eigen::Vector2f vel_avoid_nei;
    float matlab_reciver_flag_;

    std_msgs::UInt8 matlab_ugv_cmd_mode_;
    geometry_msgs::Point matlab_ugv_cmd;
    prometheus_msgs::UGVCommand ugv_command;


    // 订阅
    ros::Subscriber command_sub;
    ros::Subscriber ugv_state_sub;
    ros::Subscriber matlab_ugv_cmd_mode_sub;
    ros::Subscriber matlab_ugv_cmd_sub;

    
    // 发布
    ros::Publisher cmd_pub;
    ros::Publisher matlab_ugv_cmd_mode_pub;

    double dist,direct,yaw_error;
    bool only_rotate;
    bool first_arrive;  // 在距离上到达并停下之前都为false
    bool stop_flag;

    float test_time;

    float delta_t{0.1};
    float ugv_arr_circle_t_{0};

 enum MatlabUGVState
    {
        HOLD = 0,
        Direct_Control_BODY = 1,
        Direct_Control_ENU = 2,
        Point_Control = 3,
        Path_Control = 4,
        Test = 5
    };


private:
    void ugv_command_cb(const prometheus_msgs::UGVCommand::ConstPtr& msg);
    void ugv_state_cb(const prometheus_msgs::UGVState::ConstPtr& msg);
    void matlab_ugv_cmd_mode_cb(const std_msgs::UInt8::ConstPtr &msg);
    void matlab_ugv_cmd_cb(const geometry_msgs::Point::ConstPtr &msg);


    // void add_apf_vel();
    //Eigen::Vector3d around_Circle_trajectory(float time_from_start, int id, float lin_vel, float circle_r, int ugv_num);
    void printf_state(const ros::TimerEvent &e);
    int check_failsafe();
    void ReachTargetPoint();
    void CalErrorYaw();
    void VelLimit();

};

Eigen::Vector3d around_Circle_trajectory(float time_from_start, int id, float lin_vel, float circle_r, int ugv_num)
{
    float init_angle;
    float omega;
    float direction = -1;
    Eigen::Vector3d ugv_circle_pos_;
    omega = fabs(lin_vel / circle_r);

    init_angle = (id * 2 - 2.0) / ugv_num * M_PI; 

    const float angle = time_from_start * omega *  direction;
    const float cos_angle = cos(init_angle + angle);
    const float sin_angle = sin(init_angle + angle);

    ugv_circle_pos_[0] = circle_r * cos_angle + 0.5;
    ugv_circle_pos_[1] = circle_r * sin_angle + 0.0;
    ugv_circle_pos_[2] = 0.0;
    return ugv_circle_pos_;
}
#endif
