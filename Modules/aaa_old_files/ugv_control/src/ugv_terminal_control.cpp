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
geometry_msgs::PoseStamped goal;
ros::Publisher command_pub,goal_pub;
float state_desired[3];
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    // 无人机编号 1号无人机则为1
    nh.param("ugv_id", ugv_id, 0);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 【发布】控制指令
    command_pub = nh.advertise<prometheus_msgs::UgvCommand>(ugv_name + "/prometheus/ugv_command", 10);
    //【发布】目标点
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(ugv_name + "/prometheus/case2/goal", 10);

    // Waiting for input
    int start_flag = 0;
    
    while (ros::ok()) 
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UGV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the action: 0 for Hold, 1 for Direct Control, 2 for Point Control, 9 for astar goal pub..."<<endl;
        cin >> start_flag;
        if (start_flag == 0)
        {
            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Hold;
            command_pub.publish(ugv_cmd);
        }
        else if (start_flag == 1)
        {
            cout << "Move in Direct Control, Pls input the desired velocity and angular  velocity "<<endl;
            cout << "desired state: --- linear_vel_x(body) [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- linear_vel_y(body) [m/s] "<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- angular_vel(body) [deg/s]"<<endl;
            cin >> state_desired[2];

            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Direct_Control;
            ugv_cmd.linear_vel[0] = state_desired[0];
            ugv_cmd.linear_vel[1] = state_desired[1];
            ugv_cmd.angular_vel = state_desired[2]/180.0*M_PI;
            command_pub.publish(ugv_cmd);
            cout << "state_desired [linear angular] : " << state_desired[0] << " [m/s] "<< state_desired[1] << " [m/s] "<< state_desired[2] <<" [deg/s] "<< endl;
        } 
        else if (start_flag == 2)
        {
            cout << "Move in Point control, Pls input the desired position"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];

            ugv_cmd.Mode = prometheus_msgs::UgvCommand::Point_Control;
            ugv_cmd.position_ref[0] = state_desired[0];
            ugv_cmd.position_ref[1] = state_desired[1];
            ugv_cmd.yaw_ref = 0.0;
            command_pub.publish(ugv_cmd);
            cout << "state_desired [X Y] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< endl;
        }
        else if (start_flag == 9)
        {
            cout << "Please input the goal position:"<<endl;
            cout << "goal - x [m] : "<< endl;
            cin >> state_desired[0];
            cout << "goal -  y [m] : "<< endl;
            cin >> state_desired[1];

            goal.header.stamp =ros::Time::now();
            goal.header.frame_id = "map";
            goal.pose.position.x = state_desired[0];
            goal.pose.position.y = state_desired[1];
            goal.pose.position.z = 0.0;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            goal_pub.publish(goal);

            cout << "state_desired [X Y] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< endl;
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        ros::Duration(0.5).sleep();
    }
    return 0;
}