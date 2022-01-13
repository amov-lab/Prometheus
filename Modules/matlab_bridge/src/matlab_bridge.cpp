//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <geometry_msgs/Pose.h>

#include "printf_utils.h"

using namespace std;
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
geometry_msgs::Pose test_msg1;
geometry_msgs::Pose test_msg2;

void cmd_cb(const prometheus_msgs::UAVCommand::ConstPtr& msg)
{
    uav_command = *msg;
}
void test_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    test_msg1 = *msg;
}
void printf_msgs();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "matlab_bridge");
    ros::NodeHandle nh("~");
    
    int uav_id = 1;

    ros::Subscriber uav_command_sub = nh.subscribe<prometheus_msgs::UAVCommand>("/uav"+std::to_string(uav_id)+ "/prometheus/command", 1, cmd_cb);
    ros::Subscriber geometry_msg_sub = nh.subscribe<geometry_msgs::Pose>("/uav"+std::to_string(uav_id)+ "/prometheus/test1", 1, test_cb);

    ros::Publisher agent_state_pub = nh.advertise<prometheus_msgs::UAVState>("/uav"+std::to_string(uav_id)+"/prometheus/state", 1);
    ros::Publisher geometry_msg_pub = nh.advertise<geometry_msgs::Pose>("/uav"+std::to_string(uav_id)+"/prometheus/test2", 1);

    uav_state.header.stamp = ros::Time::now();
    uav_state.uav_id = uav_id;
    uav_state.state = prometheus_msgs::UAVState::ready;
    uav_state.mode = "OFFBOARD";
    uav_state.connected = true;
    uav_state.armed = true;
    uav_state.odom_valid = true;
    uav_state.position[0] = 1.0;
    uav_state.position[1] = 2.0;
    uav_state.position[2] = 3.0;
    uav_state.velocity[0] = 4.0;
    uav_state.velocity[1] = 5.0;
    uav_state.velocity[2] = 6.0;
    uav_state.attitude[0] = 7.0;
    uav_state.attitude[1] = 8.0;
    uav_state.attitude[2] = 9.0;

    test_msg2.position.x = 1.0;
    test_msg2.position.y = 2.0;
    test_msg2.position.z = 3.0;

    test_msg2.orientation.x = 0.0;
    test_msg2.orientation.y = 0.0;
    test_msg2.orientation.z = 0.0;
    test_msg2.orientation.w = 1.0;
    // 发布指令初始值
    
    
    while(ros::ok())
    {
        ros::spinOnce();

        agent_state_pub.publish(uav_state);
        geometry_msg_pub.publish(test_msg2);

        printf_msgs();

        ros::Duration(0.5).sleep();
    }
    return 0;
}

void printf_msgs()
{
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

    cout << GREEN  << "test_msg2:" << TAIL<<endl;
    cout << GREEN  << "position.x: " << test_msg2.position.x<<" [ m ] "<< TAIL<<endl;
    cout << GREEN  << "position.y: " << test_msg2.position.y<<" [ m ] "<< TAIL<<endl;
    cout << GREEN  << "position.z: " << test_msg2.position.z<<" [ m ] "<< TAIL<<endl;

    cout << GREEN  << "uav_command:" << TAIL<<endl;
    // 打印 指令信息
    switch(uav_command.Agent_CMD)
    {
        case prometheus_msgs::UAVCommand::Init_Pos_Hover:
            cout << GREEN  << "Command: [ Init_Pos_Hover ] " << TAIL<<endl;
            break;

        case prometheus_msgs::UAVCommand::Current_Pos_Hover:
            cout << GREEN  << "Command: [ Current_Pos_Hover ] " << TAIL<<endl;
            break;

        case prometheus_msgs::UAVCommand::Land:
            cout << GREEN  << "Command: [ Land ] " << TAIL<<endl;
            break;

        case prometheus_msgs::UAVCommand::Move:

            if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
            {
                cout << GREEN  << "Command: [ Move in XYZ_POS ] " << TAIL<<endl;
                cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS)
            {
                cout << GREEN  << "Command: [ Move in XY_VEL_Z_POS ] " << TAIL<<endl;
                cout << GREEN  << "Pos_ref [    Z] : " << uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
                cout << GREEN  << "Vel_ref [X Y  ] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL)
            {
                cout << GREEN  << "Command: [ Move in XYZ_VEL ] " << TAIL<<endl;
                cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
            {
                cout << GREEN  << "Command: [ Move in TRAJECTORY ] " << TAIL<<endl;
                cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
                cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS_BODY)
            {
                cout << GREEN  << "Command: [ Move in XYZ_POS_BODY ] " << TAIL<<endl;
                cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL_BODY)
            {
                cout << GREEN  << "Command: [ Move in XYZ_VEL_BODY ] " << TAIL<<endl;
                cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY)
            {
                cout << GREEN  << "Command: [ Move in XY_VEL_Z_POS_BODY ] " << TAIL<<endl;
                cout << GREEN  << "Pos_ref [    Z] : " << uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
                cout << GREEN  << "Vel_ref [X Y  ] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< TAIL<<endl;
                cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
            }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_ATT)
            {
                cout << GREEN  << "Command: [ Move in XYZ_ATT ] " << TAIL<<endl;
                cout << GREEN  << "Att_ref [X Y Z] : " << uav_command.att_ref[0] * 180/M_PI<< " [deg] "<< uav_command.att_ref[1]* 180/M_PI<<" [deg] "<< uav_command.att_ref[2]* 180/M_PI<<" [deg] "<< TAIL<<endl;
                cout << GREEN  << "Thrust_ref[0-1] : " << uav_command.att_ref[3] << TAIL<<endl;
            }else
            {
                cout << GREEN  << "Command: [ Unknown Mode ]. " << TAIL<<endl;
            }
            break;
    }

}