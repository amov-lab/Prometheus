/*******************************************************************
 * 文件名:formation_move.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.16
 * 
 * 介绍:该cpp文件主要为动捕集群中终端输入控制命令相关函数的实现以及程序的运行
 * ****************************************************************/

#include "Formation.h"
#include "unistd.h"

void formation::init()
{
    //创建控制命令发布者
    cmd_pub = n.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
}

void formation::move()
{
    //初始化
    init();
    while(ros::ok())
    {
        //打印提示语句,获取用户输入的控制命令
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
        std::cout << "Please input control command in LOCAL_FRAME: [x]m [y]m [z]m [yaw]radian" << std::endl;
        //X轴坐标,单位米
        std::cout << "input [x]_m" << std::endl;
        std::cin >> control_data.Reference_State.position_ref[0];
        //Y轴坐标,单位米
        std::cout << "input [y]_m" << std::endl;
        std::cin >> control_data.Reference_State.position_ref[1];
        //Z轴坐标,单位米
        std::cout << "input [z]_m" << std::endl;
        //偏航,单位度
        std::cin >> control_data.Reference_State.position_ref[2];
        std::cout << "input [yaw]_radian" << std::endl;
        std::cin >> control_data.Reference_State.yaw_ref;
        std::cout << "---------------------" << std::endl;
        std::cout << "control command: " << std::endl
                  << "x is [" << control_data.Reference_State.position_ref[0] << "]" << std::endl
                  << "y is [" << control_data.Reference_State.position_ref[1] << "]" << std::endl
                  << "z is [" << control_data.Reference_State.position_ref[2] << "]" << std::endl
                  << "yaw is [" << control_data.Reference_State.yaw_ref << "]" << std::endl;
        std::cout << "---------------------" << std::endl;
        sleep(1);
        //检查确认控制命令是否正确
        std::cout << "Please check whether the control command is correct, input 0 for continue, input 1 for input again" << std::endl;
        int check_flag;
        std::cin >> check_flag;
        if(check_flag == 1)
        {
            //打印重新输入的提示信息
            ROS_WARN("Input error , input again");
            std::cout << std::endl;
            sleep(1);
            //退出当前循环
            continue;
        }
        if(check_flag == 0)
        {
            //将控制命令的其他变量进行填充
            control_data.header.stamp = ros::Time::now();
            control_data.Mode = prometheus_msgs::ControlCommand::Move;
            control_data.source = "formation_move";
            control_data.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            //发布控制命令数据
            cmd_pub.publish(control_data);
            ROS_INFO("Control command sent");
            std::cout << std::endl;
        }
        
    }
}

int main(int argc, char** argv)
{
    //ROS初始化
    ros::init(argc, argv, "formation_move");
    //创建类并执行move函数
    formation Formation;
    Formation.move();
    return 0;
}