//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define NODE_NAME "swarm_terminal_control"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string uav_name;
int uav_id;
prometheus_msgs::SwarmCommand swarm_command;
ros::Publisher command_pub;
float state_desired[4];
bool sim_mode;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    // 无人机编号 1号无人机则为1
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("uav_name", uav_name, "/uav0");
    nh.param<bool>("sim_mode", sim_mode, true);

    command_pub = nh.advertise<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10);

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
    
    if(sim_mode)
    {
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please enter 1 to disarm the UAV and switch to OFFBOARD mode."<<endl;
            cin >> start_flag;

            swarm_command.Mode = prometheus_msgs::SwarmCommand::Idle;
            swarm_command.yaw_ref = 999;
            //【发布】阵型
            command_pub.publish(swarm_command);
        }
    }

    start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff the UAV."<<endl;
        cin >> start_flag;

        swarm_command.Mode = prometheus_msgs::SwarmCommand::Takeoff;
        swarm_command.yaw_ref = 0.0;
        //【发布】阵型
        command_pub.publish(swarm_command);
    }
    
    while (ros::ok()) 
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the action: 1 for Move(XYZ_POS), 2 for Move(XY_VEL_Z_POS), 3 for Hold, 4 for Land, 5 for Disarm..."<<endl;
        cin >> start_flag;
        if (start_flag == 1)
        {
            cout << "Move in ENU frame, Pls input the desired position and yaw angle"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]"<<endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:"<<endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3]/180.0*M_PI;
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Move;
            swarm_command.Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
            swarm_command.position_ref[0] = state_desired[0];
            swarm_command.position_ref[1] = state_desired[1];
            swarm_command.position_ref[2] = state_desired[2];
            swarm_command.yaw_ref = state_desired[3];
            //【发布】阵型
            command_pub.publish(swarm_command);

            cout << "state_desired [X Y Z] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "state_desired [YAW]: " << state_desired[3]/M_PI*180.0 <<" [ deg ] "<< endl;
        }
        else if (start_flag == 2)
        {
            cout << "Move in ENU frame, Pls input the desired velocity, position and yaw angle"<<endl;
            cout << "desired state: --- x [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]"<<endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:"<<endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3]/180.0*M_PI;
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Move;
            swarm_command.Move_mode = prometheus_msgs::SwarmCommand::XY_VEL_Z_POS;
            swarm_command.velocity_ref[0] = state_desired[0];
            swarm_command.velocity_ref[1] = state_desired[1];
            swarm_command.position_ref[2] = state_desired[2];
            swarm_command.yaw_ref = state_desired[3];
            //【发布】阵型
            command_pub.publish(swarm_command);

            cout << "state_desired [X Y Z] : " << state_desired[0] << " [ m/s ] "<< state_desired[1] <<" [ m/s ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "state_desired [YAW]: " << state_desired[3]/M_PI*180.0 <<" [ deg ] "<< endl;
            
        } 
        else if (start_flag == 3)
        {
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Hold;
            //【发布】阵型
            command_pub.publish(swarm_command);
        }
        else if (start_flag == 4)
        {
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Land;
            //【发布】阵型
            command_pub.publish(swarm_command);
        }
        else if (start_flag == 5)
        {
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Disarm;
            //【发布】阵型
            command_pub.publish(swarm_command);
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        ros::Duration(1.0).sleep();
    }
    return 0;
}