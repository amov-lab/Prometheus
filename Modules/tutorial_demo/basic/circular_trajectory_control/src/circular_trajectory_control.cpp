/******************************************************************************
*例程简介: 讲解如何调用uav_control的接口实现无人机ENU坐标系下的XY速度Z位置控制以及
*         轨迹控制
*
*效果说明: 无人机起飞后开始按照圆的轨迹飞行,飞行结束后悬停30秒,然后降落
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/


#include <ros/ros.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Eigen>
#include "printf_utils.h"

using namespace std;

const float PI = 3.1415926;

//创建无人机相关数据变量
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;
//创建圆形跟踪的相关变量
//整个圆形的飞行时间
float circular_time;
//每次控制数据更新时的弧度增量
float angle_increment;
//无人机的合速度也就是圆的线速度
float velocity;
//无人机的控制频率
float control_rate;
//圆的半径
float radius;

//通过设定整个圆的飞行时间,控制频率,圆的半径来获取相关参数
void get_circular_property(float time, int rate, float radius)
{
    //计算角速度(rad/s)
    float w = 2*PI/time;
    //计算线速度(m/s)
    velocity = radius * w;
    //计算控制数据发布的弧度增量
    angle_increment = w/rate;
}



//无人机状态回调函数
void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

//无人机控制状态回调函数
void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "body_xyz_pos_control");
    //创建句柄
    ros::NodeHandle n;
    //获取无人机id
    int uav_id;
    ros::param::get("~uav_id", uav_id);
    
    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 10);
    //创建无人机状态命令订阅者
    ros::Subscriber uav_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state", 10, uav_state_cb);
    //创建无人机控制状态命令订阅者
    ros::Subscriber uav_control_state_sub = n.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(uav_id) + "/prometheus/control_state", 10, uav_control_state_cb);
    //循环频率设置为1HZ
    ros::Rate r(1);
    //创建命令发布标志位,命令发布则为true;初始化为false
    bool cmd_pub_flag = false;

    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    
    //打印demo相关信息
    cout << GREEN << " [circular trajectory control] tutorial_demo start " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Level: [Basic] " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Please use the RC SWA to armed, and the SWB to switch the drone to [COMMAND_CONTROL] mode  " << TAIL << endl;

    //设定飞行时间为40S,控制频率为20HZ,半径为2米
    circular_time = 40;
    control_rate = 20;
    radius = 2;

    get_circular_property(circular_time, control_rate, radius);

    bool circular_success = false;
    int count = 0;
    ros::Rate rate(control_rate);

    Eigen::Vector3d target_pos = {radius, 0.0, 2.0};

    while(ros::ok())
    {
        //调用一次回调函数
        ros::spinOnce();
        //检测无人机是否处于[COMMAND_CONTROL]模式
        if(uav_control_state.control_state ==  prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            //检测控制命令是否发布,没有发布则进行命令的发布
            if(!cmd_pub_flag)
            {
                //时间戳
                uav_command.header.stamp = ros::Time::now();
                //坐标系
                uav_command.header.frame_id = "ENU";
                //Move模式
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                //Move_mode
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                //无人机将会以当前位置移动
                uav_command.position_ref[0] = target_pos[0];
                uav_command.position_ref[1] = target_pos[1];
                uav_command.position_ref[2] = target_pos[2];
                uav_command.yaw_ref = 0.0;
                //发布的命令ID加1
                uav_command.Command_ID += 1;
                //发布降落命令
                uav_command_pub.publish(uav_command);
                //命令发布标志位置为true
                cmd_pub_flag = true;

                //打印无人机控制命令发布信息
                cout << GREEN << " Go to initial point " << TAIL << endl;

                //检测无人机是否到达初始点
                bool initial_point_success = false;
                while(!initial_point_success)
                {
                    ros::spinOnce();
                    //当无人机距离目标位置±0.1米范围内时认为到达初始点
                    Eigen::Vector3d uav_pos = {uav_state.position[0], uav_state.position[1], uav_state.position[2]};
                    float distance = (uav_pos - target_pos).norm();
                    if(distance < 0.1)
                    {
                        cout << GREEN << " UAV arrived at initial point " << TAIL << endl;
                        initial_point_success = true;
                    }
                    r.sleep();
                }
            }
            else
            {
                //时间戳
                uav_command.header.stamp = ros::Time::now();
                //坐标系
                uav_command.header.frame_id = "ENU";
                //Move模式
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                //Move_mode
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
                //无人机按照圆形轨迹飞行
                uav_command.velocity_ref[0] = -velocity*sin(count*angle_increment);
                uav_command.velocity_ref[1] = velocity*cos(count*angle_increment);
                uav_command.velocity_ref[2] = 0;
                uav_command.position_ref[2] = target_pos[2];
                //发布的命令ID加1
                uav_command.Command_ID += 1;
                //发布降落命令
                uav_command_pub.publish(uav_command);
                //计数器
                if(count == control_rate*circular_time)
                {
                    circular_success = true;
                }
                count++;
                cout << GREEN << " Vx: " << uav_command.velocity_ref[0] << TAIL << endl;
                cout << GREEN << " Vy: " << uav_command.velocity_ref[1] << TAIL << endl;
            
                if(circular_success)
                {
                    cout << GREEN << " UAV circular trajectory completed and landed after 30 seconds" << TAIL << endl;
                    //时间戳
                    uav_command.header.stamp = ros::Time::now();
                    //坐标系
                    uav_command.header.frame_id = "ENU";
                    //Land降落,从当前位置降落至地面并自动上锁
                    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                    //发布的命令ID加1
                    uav_command.Command_ID += 1;
                    //发布降落命令
                    uav_command_pub.publish(uav_command);
                    sleep(30);
                    //时间戳
                    uav_command.header.stamp = ros::Time::now();
                    //坐标系
                    uav_command.header.frame_id = "ENU";
                    //Land降落,从当前位置降落至地面并自动上锁
                    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                    //发布的命令ID加1
                    uav_command.Command_ID += 1;
                    //发布降落命令
                    uav_command_pub.publish(uav_command);
                    //打印降落相关信息
                    cout << GREEN << " UAV Land" << TAIL << endl;
                    cout << GREEN << " [circular trajectory control] tutorial_demo completed" << TAIL << endl;
                    //任务结束,关闭该节点
                    ros::shutdown();
                }
            }
        }
        else
        {
            //在控制命令发布后,但无人机未结束任务的情况下,此时无人机未处于[COMMAND_CONTROL]控制状态,认为无人机出现意外情况,任务中止
            if(cmd_pub_flag)
            {
                cout << RED << " Unknown error! [ENU XYZ position control] tutorial_demo aborted" << TAIL << endl;
                r.sleep();
                continue;
            }
            //命令未发布,等待无人机进入[COMMAND_CONTROL]状态
            else
            {
                cout << YELLOW << " Wait for UAV to enter [COMMAND_CONTROL] MODE " << TAIL << endl;
                r.sleep();
                continue;
            }
        }
        rate.sleep();
    }
    return 0;
}