#include <ros/ros.h>
#include <iostream>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "ros_info_color_test"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_info_color_test");
    ros::NodeHandle nh;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);


    ros::Rate rate(10);

    while(ros::ok())
    {


        cout << ">>>>>>>>>>>>>>>> Welcome to use ros_info_color_test <<<<<<<<<<<<<<<<"<< endl;

        // 基本
        ROS_INFO("---->ROS_INFO");
        ROS_WARN("---->ROS_WARN");
        ROS_ERROR("---->ROS_ERROR");

        // 仅打印一次
        ROS_INFO_STREAM_ONCE ("---->This message only printf once.");

        // 按频率打印
        float interval = 1.0;
        ROS_INFO_STREAM_THROTTLE(interval, "---->This message  printf in 1 Hz.");


        // printf(“\033[0;30;41m color!!! \033[0m Hello \n”);
         // 其中41的位置代表字的背景色, 30的位置是代表字的颜色，0 是字的一些特殊属性，0代表默认关闭，一些其他属性如闪烁、下划线等。     
        // 参考网页：https://blog.csdn.net/u014470361/article/details/81512330

        // 字背景颜色范围:40--49                   字颜色: 30--39  
        // 40: 黑                         30: 黑  
        // 41:红                          31: 红  
        // 42:绿                          32: 绿  
        // 43:黄                          33: 黄  
        // 44:蓝                          34: 蓝  
        // 45:紫                          35: 紫  
        // 46:深绿                        36: 深绿  
        // 47:白色                        37: 白色  


        // 绿色字体
        ROS_INFO("\033[1;31m----> Red color.\033[0m");
        ROS_INFO("\033[1;32m----> Green color.\033[0m");
        ROS_INFO("\033[1;33m----> Yellow color.\033[0m");

        ROS_INFO("\033[1;33;41m----> Hightlight color.\033[0m");

        // 打印数值
        float a = 10.0;
        ROS_INFO("---->The value of a is  %f.", a);

        cout << "\033[1;31m" << "Hello World, Red color!"  << "\033[0m" << endl;
        cout << "\033[1;32m" << "Hello World, Green color!"  << "\033[0m" << endl;
        cout << "\033[1;33m" << "Hello World, Yellow color!"  << "\033[0m" << endl;

        string yellow, tail;
        yellow = "\033[0;1;33m";
        tail =  "\033[0m";
        cout << yellow << "hello world"  << tail << endl;


        rate.sleep();
    }


    return 0;
}
