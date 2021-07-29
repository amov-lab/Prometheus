/*******************************************************************
 * 文件名:formation_change.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.14
 * 
 * 介绍:该cpp文件主要为动捕集群中模式切换相关函数的实现以及程序的运行
 * ****************************************************************/
#include "Formation.h"
#include <iostream>

void formation::init()
{
    //创建队形变化数据发布者
    ros::param::param<int>("~DIAMOND_intervals", diamond_intervals, 5);
    ros::param::param<string>("Location_source", location_source, "gps");
    formation_type_pub = n.advertise<prometheus_msgs::Formation>("/prometheus/formation/change", 10);
}

//是否等待函数
void formation::is_wait(int time)
{
    //判断是否需要等待,time变量不为0,则等待
    if(time != 0)
    {
        sleep(time);
    }

}

//打印集群队形状态函数
void formation::printf_formation_type(std::string type_name)
{
    std::cout << std::endl;
    std::cout <<  ">>>>>>>>Formation Type<<<<<<<<" << std::endl;
    std::cout << std::endl;
    std::cout <<  "        " << type_name << "        " << std::endl;
    std::cout << std::endl;
    std::cout <<  "------------------------------" << std::endl;
    std::cout << std::endl;
}

void formation::change()
{
    //初始化,创建队形变换信息发布者
    init();
    //创建获取用户输入值的变量
    int type_last; 
    int type_now;

    while(ros::ok())
    {
        //打印提示信息
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
        std::cout << "Input the formation type:  0 for horizontal formation, 1 for triangle formation, 2 for diamond formation" << std::endl;
        //获取用户输入的值
        std::cin >> type_now;
        //判断值是否正确,正确则进入正常流程,错误则打印警告信息
        if((type_now >= 0) && (type_now <= 2))
        {
            switch(type_now)
            {
                //一字队形
                case 0:
                    //判断当前队形是否为菱形队形,是菱形则先进入过渡队形
                    if(type_last == prometheus_msgs::Formation::DIAMOND)
                    {
                        formation_data.type = prometheus_msgs::Formation::DIAMOND_STAGE_1;
                        formation_type_pub.publish(formation_data);
                        is_wait(diamond_intervals);
                    }
                    //切换为一字队形
                    formation_data.type = prometheus_msgs::Formation::HORIZONTAL;
                    formation_type_pub.publish(formation_data);
                    printf_formation_type("Horizontal");
                    break;
                
                //三角队形
                case 1:
                    //判断当前队形是否为菱形队形,是菱形则先进入过渡队形
                    if(type_last == prometheus_msgs::Formation::DIAMOND)
                    {
                        formation_data.type = prometheus_msgs::Formation::DIAMOND_STAGE_1;
                        formation_type_pub.publish(formation_data);
                        is_wait(diamond_intervals);
                    }
                    //切换为三角队形
                    formation_data.type = prometheus_msgs::Formation::TRIANGEL;
                    formation_type_pub.publish(formation_data);
                    printf_formation_type("Triangle");
                    break;

                //菱形队形
                case 2:
                    if(location_source == "mocap")
                    {
                        //判断当前队形是否为菱形队形,是菱形队形则直接跳出当前队形
                        if(type_last == type_now)
                        {
                            break;
                        }
                        //进入过渡队形
                        formation_data.type = prometheus_msgs::Formation::DIAMOND_STAGE_1;
                        formation_type_pub.publish(formation_data);
                        is_wait(diamond_intervals);
                        //切换为菱形队形
                        formation_data.type = prometheus_msgs::Formation::DIAMOND;
                        formation_type_pub.publish(formation_data);
                        printf_formation_type("Diamond");
                        break;
                    }
                    if(location_source == "uwb" || location_source == "gps")
                    {
                        ROS_WARN("not support diamond formation");
                        break;
                    }
                    
            }
            //把当前集群队形变量赋值给上一集群队形变量
            type_last = type_now;
        }
        else
        {
            type_now = type_last;
            //输入错误
            ROS_WARN("inpunt error, please input again");
        }
        
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_change");
    formation Formation;
    Formation.change();
    return 0;
}
