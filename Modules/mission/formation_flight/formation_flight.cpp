//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/PositionReference.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
using namespace std;


#define FIVE_STAR_SIZE 2.0
#define TRIANGLE_SIZE 2.0
#define T_SIZE 2.0
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
geometry_msgs::Point leader;
geometry_msgs::Point relative_pos_to_leader1;
geometry_msgs::Point relative_pos_to_leader2;
geometry_msgs::Point relative_pos_to_leader3;
geometry_msgs::Point relative_pos_to_leader4;
geometry_msgs::Point relative_pos_to_leader5;

geometry_msgs::PoseStamped goal;
int flag_get_goal;

ros::Publisher leader_pub;
ros::Publisher formation_pub1;
ros::Publisher formation_pub2;
ros::Publisher formation_pub3;
ros::Publisher formation_pub4;
ros::Publisher formation_pub5;

float uav_number;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void five_star();
void triangle();
void T_shape();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    flag_get_goal = 1;
    cout << "Get a new goal from rviz!"<<endl;

    //高度打死
    leader.x = goal.pose.position.x;
    leader.y = goal.pose.position.y;
    leader.z = 1.0;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_flight");
    ros::NodeHandle nh("~");

    nh.param<float>("uav_number", uav_number, 5);
    
    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/formation/goal", 10, goal_cb);

    //【发布】阵型
    leader_pub     = nh.advertise<geometry_msgs::Point>("/prometheus/formation/leader_pos", 10);
    formation_pub1 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav1", 10);
    formation_pub2 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav2", 10);
    formation_pub3 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav3", 10);
    formation_pub4 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav4", 10);
    formation_pub5 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav5", 10);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;

    ros::Duration(1.0).sleep();

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;
        leader_pub.publish(leader);
    }

    float x_sp,y_sp;
    while (ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the formation: 1 for five star, 2 for triangle, 3 for T shape..."<<endl;
        cin >> start_flag;
        cout << "Please enter the desired position (if x_sp = 999, use the position choosen in rviz):"<<endl;
        cout << "x_sp:"<<endl;
        cin >> x_sp;
        cout << "y_sp:"<<endl;
        cin >> y_sp;

        if(x_sp == 999)
        {
            ros::spinOnce();
        }else{
            //高度打死
            leader.x = x_sp;
            leader.y = y_sp;
            leader.z = 1.0;
        }

        //发布
        leader_pub.publish(leader);
        
        //选择阵型
        if (start_flag == 1)
        {
            five_star();
        }else if (start_flag == 2)
        {
            triangle();
        }else if (start_flag == 3)
        {
            T_shape();
        }
        

        cout << "Leader [X Y Z] : " << leader.x << " [ m ] "<< leader.y <<" [ m ] "<< leader.z <<" [ m ] "<<endl;
     
        ros::Duration(2.0).sleep();
    }

    return 0;

}

void five_star()
{   
    relative_pos_to_leader1.x = FIVE_STAR_SIZE;
    relative_pos_to_leader1.y = 0.0;
    relative_pos_to_leader1.z = 0.0;
    formation_pub1.publish(relative_pos_to_leader1);

    relative_pos_to_leader2.x = 0.0;
    relative_pos_to_leader2.y = FIVE_STAR_SIZE;
    relative_pos_to_leader2.z = 0.0;
    formation_pub2.publish(relative_pos_to_leader2);

    relative_pos_to_leader3.x = -FIVE_STAR_SIZE;
    relative_pos_to_leader3.y = 0.5*FIVE_STAR_SIZE;
    relative_pos_to_leader3.z = 0.0;
    formation_pub3.publish(relative_pos_to_leader3);

    relative_pos_to_leader4.x = -FIVE_STAR_SIZE;
    relative_pos_to_leader4.y = -0.5*FIVE_STAR_SIZE;
    relative_pos_to_leader4.z = 0.0;
    formation_pub4.publish(relative_pos_to_leader4);

    relative_pos_to_leader5.x = 0.0;
    relative_pos_to_leader5.y = -FIVE_STAR_SIZE;
    relative_pos_to_leader5.z = 0.0;
    formation_pub5.publish(relative_pos_to_leader5);
}

void triangle()
{   
    relative_pos_to_leader1.x = TRIANGLE_SIZE;
    relative_pos_to_leader1.y = TRIANGLE_SIZE;
    relative_pos_to_leader1.z = 0.0;
    formation_pub1.publish(relative_pos_to_leader1);

    relative_pos_to_leader2.x = 0.0;
    relative_pos_to_leader2.y = 2 * TRIANGLE_SIZE;
    relative_pos_to_leader2.z = 0.0;
    formation_pub2.publish(relative_pos_to_leader2);

    relative_pos_to_leader3.x = 0.0;
    relative_pos_to_leader3.y = - 2 * TRIANGLE_SIZE;
    relative_pos_to_leader3.z = 0.0;
    formation_pub3.publish(relative_pos_to_leader3);

    relative_pos_to_leader4.x = TRIANGLE_SIZE;
    relative_pos_to_leader4.y = -TRIANGLE_SIZE;
    relative_pos_to_leader4.z = 0.0;
    formation_pub4.publish(relative_pos_to_leader4);

    relative_pos_to_leader5.x = 2 * TRIANGLE_SIZE;
    relative_pos_to_leader5.y = 0.0;
    relative_pos_to_leader5.z = 0.0;
    formation_pub5.publish(relative_pos_to_leader5);
}

void T_shape()
{
    relative_pos_to_leader1.x = 2 * T_SIZE;
    relative_pos_to_leader1.y = - T_SIZE;
    relative_pos_to_leader1.z = 0.0;
    formation_pub1.publish(relative_pos_to_leader1);

    relative_pos_to_leader2.x = 2 * T_SIZE;
    relative_pos_to_leader2.y = 0.0;
    relative_pos_to_leader2.z = 0.0;
    formation_pub2.publish(relative_pos_to_leader2);

    relative_pos_to_leader3.x = 2 * T_SIZE;
    relative_pos_to_leader3.y = T_SIZE;
    relative_pos_to_leader3.z = 0.0;
    formation_pub3.publish(relative_pos_to_leader3);

    relative_pos_to_leader4.x = T_SIZE;
    relative_pos_to_leader4.y = 0.0;
    relative_pos_to_leader4.z = 0.0;
    formation_pub4.publish(relative_pos_to_leader4);

    relative_pos_to_leader5.x = 0.0;
    relative_pos_to_leader5.y = 0.0;
    relative_pos_to_leader5.z = 0.0;
    formation_pub5.publish(relative_pos_to_leader5);
}