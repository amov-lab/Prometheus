/******************************************************************************
*例程简介: 讲解如何调用uav_control的接口实现无人机ENU坐标系下的位置控制
*
*效果说明: 无人机起飞后悬停30秒,然后降落
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/
#include <ros/ros.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <unistd.h>
#include "printf_utils.h"

using namespace std;

//创建无人机相关数据变量
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;

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
    ros::init(argc , argv, "takeoff_land");
    //创建句柄
    ros::NodeHandle n;
    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav1/prometheus/command", 10);
    //创建无人机状态命令订阅者
    ros::Subscriber uav_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav1/prometheus/state", 10, uav_state_cb);
    //创建无人机控制状态命令订阅者
    ros::Subscriber uav_control_state_sub = n.subscribe<prometheus_msgs::UAVControlState>("/uav1/prometheus/control_state", 10, uav_control_state_cb);
    //循环频率设置为1HZ
    ros::Rate r(1);
    //创建命令发布标志位,命令发布则为true;初始化为false
    bool cmd_pub_flag = false;
    //声明起飞高度变量
    float takeoff_height;

    //打印demo相关信息
    cout << GREEN << " [takeoff & land] tutorial_demo start " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Level: [Basic] " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Please use the RC SWA to armed, and the SWB to switch the drone to [COMMAND_CONTROL] mode  " << TAIL << endl;

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
                //Init_Pos_Hover初始位置悬停,可在uav_control_indoor.yaml或uav_control_outdoor.yaml文件设置无人机悬停高度
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                //发布的命令ID,每发一次,该ID加1
                uav_command.Command_ID = 1;
                //发布起飞命令
                uav_command_pub.publish(uav_command);
                //命令发布标志位置为true
                cmd_pub_flag = true;
                //获取起飞高度参数
                ros::param::get("/uav_control_main_1/control/Takeoff_height", takeoff_height);

                //打印无人机起飞相关信息
                cout << GREEN << " [Init_Pos_Hover] command publish " << TAIL << endl;
                cout << GREEN << " UAV takeoff " << TAIL << endl;
                cout << GREEN << " Takeoff_height: " << takeoff_height << " [m]" << TAIL << endl;
            }
            else
            {
                //当无人机距离高度目标值±0.3米范围内时认为起飞完成并等待30秒后降落
                if(fabs(uav_state.position[2] - takeoff_height) <= 0.3)
                {
                    cout << GREEN << " UAV takeoff successfully and landed after 30 seconds" << TAIL << endl;
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
                    cout << GREEN << " [takeoff & land] tutorial_demo completed" << TAIL << endl;
                    //任务结束,关闭该节点
                    ros::shutdown();
                }
                else
                {
                    //打印当前无人机高度信息
                    cout << GREEN << " UAV height: " << uav_state.position[2] << " [m]" << TAIL << endl;
                }
            }
        }
        else
        {
            //在控制命令发布后,但无人机未结束任务的情况下,此时无人机未处于[COMMAND_CONTROL]控制状态,认为无人机出现意外情况,任务中止
            if(cmd_pub_flag)
            {
                cout << RED << " Unknown error! [takeoff & land] tutorial_demo aborted" << TAIL << endl;
            }
            //命令未发布,等待无人机进入[COMMAND_CONTROL]状态
            else
            {
                cout << YELLOW << " Wait for UAV to enter [COMMAND_CONTROL] MODE " << TAIL << endl;
            }
        } 
        r.sleep();
    }
    return 0;
}