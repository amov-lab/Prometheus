//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/UgvCommand.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define NODE_NAME "ugv_terminal_control"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string ugv_name;
int ugv_id;
prometheus_msgs::UgvCommand ugv_cmd;
ros::Publisher command_pub;
float state_desired[3];
bool sim_mode;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    // 无人机编号 1号无人机则为1
    nh.param<string>("ugv_name", ugv_name, "/ugv0");
    nh.param<bool>("sim_mode", sim_mode, true);

    command_pub = nh.advertise<prometheus_msgs::UgvCommand>(ugv_name + "/prometheus/ugv_command", 10);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    
    if(!sim_mode)
    {
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please enter 1 to disarm the UGV and switch to OFFBOARD mode."<<endl;
            cin >> start_flag;

            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Start;
            ugv_cmd.linear_vel[0] = 999;
            //【发布】阵型
            command_pub.publish(ugv_cmd);
        }
    }
    
    while (ros::ok()) 
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the action: 1 for Point control, 2 for Direct_Control, 3 for ENU_vel_control, 4 for Hold, 5 for Disarm..."<<endl;
        cin >> start_flag;
        if (start_flag == 1)
        {
            cout << "Move in Point control, Pls input the desired position and yaw angle"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- yaw [deg]"<<endl;
            cin >> state_desired[2];
            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Point_Control;
            ugv_cmd.position_ref[0] = state_desired[0];
            ugv_cmd.position_ref[1] = state_desired[1];
            ugv_cmd.yaw_ref = state_desired[2]/180.0*M_PI;

            //【发布】阵型
            command_pub.publish(ugv_cmd);

            cout << "state_desired [X Y] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< endl;
            cout << "state_desired [YAW] : " << state_desired[2] <<" [deg] "<< endl;
        }
        else if (start_flag == 2)
        {
            cout << "Move in Direct_Control, Pls input the desired velocity and angular  velocity "<<endl;
            cout << "desired state: --- linear_vel(body) [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- angular_vel(body) [deg/s]"<<endl;
            cin >> state_desired[1];

            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Direct_Control;
            ugv_cmd.linear_vel[0] = state_desired[0];
            ugv_cmd.angular_vel = state_desired[1]/180.0*M_PI;

            command_pub.publish(ugv_cmd);

            cout << "state_desired [linear angular] : " << state_desired[0] << " [m/s] "<< state_desired[1] <<" [deg/s] "<< endl;
        }         
        else if (start_flag == 3)
        {
            cout << "Move in ENU_vel_control, Pls input the desired velocity and yaw angle"<<endl;
            cout << "desired state: --- x [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- yaw [deg]"<<endl;
            cin >> state_desired[2];

            ugv_cmd.Mode = prometheus_msgs::UgvCommand::ENU_Vel_Control;
            ugv_cmd.linear_vel[0] = state_desired[0];
            ugv_cmd.linear_vel[1] = state_desired[1];
            ugv_cmd.yaw_ref = state_desired[2]/180.0*M_PI;

            command_pub.publish(ugv_cmd);

            cout << "state_desired [x y] : " << state_desired[0] << " [ m/s ] "<< state_desired[1] <<" [ m/s ] "<< endl;
            cout << "state_desired [YAW] : " << state_desired[2] << " [deg] "<< endl;
        }   
        else if (start_flag == 4)
        {
            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Hold;
            //【发布】阵型
            command_pub.publish(ugv_cmd);
        }
        else if (start_flag == 5)
        {
            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Disarm;
            //【发布】阵型
            command_pub.publish(ugv_cmd);
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        ros::Duration(1.0).sleep();
    }
    return 0;
}