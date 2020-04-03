/***************************************************************************************************************************
* formation_control.cpp
*
* Author: Qyp
*
* Update Time: 2020.4.2
*                           采用虚拟领机-从机结构。地面站为主节点，每一个飞机都为从机                
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <command_to_mavros.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <bitset>
#include <Eigen/Eigen>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string uav_name;
string neighbour_name1,neighbour_name2;
int num_neighbour = 2;
ros::Publisher setpoint_raw_local_pub;

mavros_msgs::State current_state;
Eigen::Vector3d pos_drone;
Eigen::Vector3d vel_drone;

Eigen::Vector3d pos_leader;
Eigen::Vector3d relative_pos_to_leader;

Eigen::Vector3d state_sp(0,0,0);
float yaw_sp = 0;
bool get_new_cmd = false;

float k_p;
float k_aij;

//mavros_msgs::State current_state_nei[2];
Eigen::Vector3d pos_nei[2];
Eigen::Vector3d vel_nei[2];
//2020.4.2 这里有个十分神奇的bug： 这个语句后面不要再放任何变量声明了，不然变量的值会各种变化。。
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void pos_leader_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    pos_leader = Eigen::Vector3d(msg->x,msg->y,msg->z);
    get_new_cmd = true;
}
void formation_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    relative_pos_to_leader  = Eigen::Vector3d(msg->x,msg->y,msg->z);
}
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_drone  = Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    vel_drone  = Eigen::Vector3d(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
}
void odom_nei_cb(const nav_msgs::Odometry::ConstPtr &msg, int nei_id)
{
    pos_nei[nei_id]  = Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    vel_nei[nei_id]  = Eigen::Vector3d(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_control");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    //无人机编号 1号无人机则为1
    nh.param<string>("uav_name", uav_name, "/uav0");
    nh.param<float>("k_p", k_p, 0.95);
    nh.param<float>("k_aij", k_aij, 0.1);
    //可监听到的无人机编号，目前设定为可监听到两台无人机，后期考虑可通过数组传递参数，监听任意ID的无人机
    nh.param<string>("neighbour_name1", neighbour_name1, "/uav0");
    nh.param<string>("neighbour_name2", neighbour_name2, "/uav0");

    //订阅虚拟主机的位置，及阵型
    ros::Subscriber leader_sub = nh.subscribe<geometry_msgs::Point>("/prometheus/formation/leader_pos", 10, pos_leader_cb);
    ros::Subscriber formation_sub = nh.subscribe<geometry_msgs::Point>("/prometheus/formation" + uav_name, 10, formation_cb);

    //订阅本台飞机的状态
    ros::Subscriber state_sub    = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/mavros/local_position/odom", 100, odom_cb);

    //订阅邻居飞机的状态信息
    ros::Subscriber odom_nei_sub_1 = nh.subscribe<nav_msgs::Odometry>(neighbour_name1 + "/mavros/local_position/odom", 100, boost::bind(&odom_nei_cb,_1, 0));   
    ros::Subscriber odom_nei_sub_2 = nh.subscribe<nav_msgs::Odometry>(neighbour_name2 + "/mavros/local_position/odom", 100, boost::bind(&odom_nei_cb,_1, 1)); 

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;

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
    cout << "Waiting for start command......"<< endl;

    int start_flag = 0;
    while(!get_new_cmd)
    {
        _command_to_mavros.idle();
        ros::spinOnce();
        rate.sleep();
    }

    get_new_cmd = false;

    cout << "Switch to OFFBOARD and arm ..."<<endl;
    while(current_state.mode != "OFFBOARD")
    {
        _command_to_mavros.mode_cmd.request.custom_mode = "OFFBOARD";
        _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
        _command_to_mavros.idle();
        //执行回调函数
        ros::spinOnce();
        ros::Duration(0.5).sleep();

        if (_command_to_mavros.mode_cmd.response.mode_sent)
        {
            cout <<"[formation_flight, UAV_name: "<<uav_name<<"]: "<< "Set to OFFBOARD Mode Susscess! "<< endl;
        }
    }

    while(!current_state.armed)
    {
        _command_to_mavros.arm_cmd.request.value = true;
        _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
        _command_to_mavros.idle();
        //执行回调函数
        ros::spinOnce();
        ros::Duration(0.5).sleep();

        if (_command_to_mavros.arm_cmd.response.success)
        {
            cout <<"[formation_flight, UAV_name: "<<uav_name<<"]: "<< "Arm Susscess! "<< endl;
        }
    }

    //起飞位置为飞机所在位置上方1米
    cout<<"[formation_flight, UAV_name: "<<uav_name<<"]: "<<"Takeoff to predefined position."<<endl;
    Eigen::Vector3d takeoff_pos = pos_drone + Eigen::Vector3d(0,0,1);
    yaw_sp = 0.0;
    while(!get_new_cmd)
    { 
        _command_to_mavros.send_pos_setpoint(takeoff_pos, yaw_sp);
        rate.sleep();
        ros::spinOnce();
    }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        //formation control， 平面阵型，xy控制速度，z轴高度定高
        state_sp[0] = k_p * (pos_leader[0] + relative_pos_to_leader[0] - pos_drone[0]) + k_aij*(pos_nei[0][0] - pos_drone[0]) + k_aij*(pos_nei[1][0] - pos_drone[0]);
        state_sp[1] = k_p * (pos_leader[1] + relative_pos_to_leader[1] - pos_drone[1]) + k_aij*(pos_nei[0][1] - pos_drone[1]) + k_aij*(pos_nei[1][1] - pos_drone[1]);
        state_sp[2] = 1.0;
        yaw_sp = 0.0;
        _command_to_mavros.send_vel_xy_pos_z_setpoint(state_sp, yaw_sp);

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "UAV_name : " << uav_name <<"   UAV_mode: "<< current_state.mode <<"   arm or not: "<< current_state.armed << endl;
        cout << "Position [X Y Z] : " << pos_drone[0] << " [ m ] "<< pos_drone[1]<<" [ m ] "<<pos_drone[2]<<" [ m ] "<<endl;
        cout << "Velocity [X Y Z] : " << vel_drone[0] << " [m/s] "<< vel_drone[1]<<" [m/s] "<<vel_drone[2]<<" [m/s] "<<endl;
        cout << "P_Leader [X Y Z] : " << pos_leader[0] << " [ m ] "<< pos_leader[1]<<" [ m ] "<<pos_leader[2]<<" [ m ] "<<endl;
        cout << "Pos_Nei1 [X Y Z] : " << pos_nei[0][0] << " [ m ] "<< pos_nei[0][1]<<" [ m ] "<<pos_nei[0][2]<<" [ m ] "<<endl;
        cout << "Pos_Nei2 [X Y Z] : " << pos_nei[1][0] << " [ m ] "<< pos_nei[1][1]<<" [ m ] "<<pos_nei[1][2]<<" [ m ] "<<endl;
        cout << "Relative [X Y Z] : " << relative_pos_to_leader[0] << " [ m ] "<< relative_pos_to_leader[1]<<" [ m ] "<<relative_pos_to_leader[2]<<" [ m ] "<<endl;
        cout << "state_sp [X Y Z] : " << state_sp[0]  << " [m/s] "<< state_sp[1] <<" [m/s] "<<state_sp[2] <<" [m/s] "<<endl;
        
        rate.sleep();
    }

}