#include "ugv_controller.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_controller");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    init(nh);

    //【订阅】无人车控制指令
    command_sub = nh.subscribe<prometheus_msgs::UgvCommand>(ugv_name + "/prometheus/ugv_command", 2, ugv_command_cb);
    //【订阅】本机状态信息
    ugv_state_sub = nh.subscribe<prometheus_msgs::UgvState>(ugv_name + "/prometheus/ugv_state", 2, ugv_state_cb);
    //【发布】底层控制指令
    cmd_pub = nh.advertise<geometry_msgs::Twist>(ugv_name + "/cmd_vel", 10);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 执行回调函数
        ros::spinOnce();

        // 一般选择不打印
        if(flag_printf)
        {
            printf_state();
        }

        // Check for geo fence: If ugv is out of the geo fence, it will hold now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = prometheus_msgs::UgvCommand::Hold;
        }

        switch (Command_Now.Mode)
        {
        // 【Start】 
        case prometheus_msgs::UgvCommand::Hold:
            
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_pub.publish(cmd_vel);
            break;

        case prometheus_msgs::UgvCommand::Direct_Control:

            // 注: linear.x与linear.y控制的是无人车车体系下的线速度
            cmd_vel.linear.x = Command_Now.linear_vel[0];
            cmd_vel.linear.y = Command_Now.linear_vel[1];
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = Command_Now.angular_vel;
            cmd_pub.publish(cmd_vel);

            break;
        case prometheus_msgs::UgvCommand::Point_Control:

            cmd_vel.linear.x = k_p*(Command_Now.position_ref[0] - pos_ugv[0]);
            cmd_vel.linear.y = k_p*(Command_Now.position_ref[1] - pos_ugv[1]);
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = k_yaw*(0.0 - yaw_ugv);

            // 速度限制幅度 
            if(cmd_vel.linear.x > max_vel)
            {
                cmd_vel.linear.x = max_vel;
            }else if(cmd_vel.linear.x < -max_vel)
            {
                cmd_vel.linear.x = -max_vel;
            }

            if(cmd_vel.linear.y > max_vel)
            {
                cmd_vel.linear.y = max_vel;
            }else if(cmd_vel.linear.y < -max_vel)
            {
                cmd_vel.linear.y = -max_vel;
            }
            cmd_pub.publish(cmd_vel);

            break;

        case prometheus_msgs::UgvCommand::Path_Control:
            // 空缺
            break;

        }

        Command_Last = Command_Now;
        rate.sleep();
    }
}