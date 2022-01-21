//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
using namespace std;

#define NODE_NAME "swarm_terminal_control"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string uav_name;
int uav_id;
prometheus_msgs::SwarmCommand swarm_command;
ros::Publisher command_pub,ego_wp_pub,goal_pub;
float state_desired[4];
bool sim_mode;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    // 无人机编号 1号无人机则为1
    nh.param("uav_id", uav_id, 0);
    nh.param("sim_mode", sim_mode, true);

    uav_name = "/uav" + std::to_string(uav_id);
    //　【发布】　指令(swarm-controller)
    command_pub = nh.advertise<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10);
    //　【发布】　参考轨迹(ego-planner)
    ego_wp_pub = nh.advertise<nav_msgs::Path>(uav_name + "/waypoint_generator/waypoints", 10);
    //　【发布】　参考轨迹(ego-planner-swarm)
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/prometheus/ego/goal", 10);

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
        cout << "Please choose the action: 1 for Move(XYZ_POS), 2 for EGO goal, 3 for Hold, 4 for Land, 5 for Disarm..."<<endl;
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

            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "yaw_des : " << state_desired[3]/M_PI*180.0 <<" [ deg ] "<< endl;
        }
        else if (start_flag == 2)
        {
            cout << "Move in ENU frame, Pls input the desired velocity, position and yaw angle"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]"<<endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:"<<endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3]/180.0*M_PI;

            geometry_msgs::PoseStamped pt;

            pt.header.stamp = ros::Time::now();
            pt.header.frame_id = std::string("world");
            pt.pose.position.x = state_desired[0];
            pt.pose.position.y = state_desired[1];
            pt.pose.position.z = state_desired[2];
            pt.pose.orientation = tf::createQuaternionMsgFromYaw(state_desired[3]/180*3.1415926);
            goal_pub.publish(pt);

            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "yaw_des : " << state_desired[3]/M_PI*180.0 <<" [ deg ] "<< endl;
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
        ros::Duration(0.5).sleep();
    }
    return 0;
}