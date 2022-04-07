//ROS 头文件
#include <ros/ros.h>
#include <iostream>

//topic 头文件
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <prometheus_msgs/DroneState.h>

using namespace std;

geometry_msgs::Pose qrcode_pose;   
geometry_msgs::Twist qrcode_twist; 
prometheus_msgs::DroneState _Drone_state;   
gazebo_msgs::ModelState set_state;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int modelCount = msg->name.size();
    for(int modelInd = 0;modelInd<modelCount;++modelInd)
    {
        if(msg->name[modelInd] == "car_landing_pad")
        {
            qrcode_pose = msg->pose[modelInd];
            qrcode_twist = msg->twist[modelInd];
            break;
        }
    }
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _Drone_state = *msg;
}
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrcode_control");
    ros::NodeHandle nh("~");

    ros::Subscriber model_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, modelStatesCallback);
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    ros::Publisher state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    set_state.model_name = "car_landing_pad";

    float time = 0;
    while (ros::ok())
    {
        float linear_vel = 0.1;
        float circle_radius = 1.0;
        float omega = fabs(linear_vel / circle_radius);

        const float angle = time * omega;
        const float cos_angle = cos(angle);
        const float sin_angle = sin(angle);

        set_state.pose.position.x = circle_radius * cos_angle + 0;
        set_state.pose.position.y = circle_radius * sin_angle + 0;
        set_state.pose.position.z = 0.05;
        // set_state.twist.linear.x = - circle_radius * omega * sin_angle;
        // set_state.twist.linear.y = circle_radius * omega * cos_angle;
        // set_state.twist.linear.z = 0;
        time = time + 0.05;
        ros::Duration(0.05).sleep();
        state_pub.publish(set_state);
        printf_result();
        ros::spinOnce();
    }

    return 0;

}


void printf_result()
{
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

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Test<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "qrcode_pose      : " << qrcode_pose.position.x << " [m]   " << qrcode_pose.position.y << " [m]   " << qrcode_pose.position.z << " [m]   "<<endl;
    cout << "qrcode_twist     : " << qrcode_twist.linear.x  << " [m/s] " << qrcode_twist.linear.y  << " [m/s] " << qrcode_twist.linear.z  << " [m/s] "<<endl;
    cout << "Position [X Y Z] : " << _Drone_state.position[0] << " [ m ] "<< _Drone_state.position[1]<<" [ m ] "<<_Drone_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << _Drone_state.velocity[0] << " [m/s] "<< _Drone_state.velocity[1]<<" [m/s] "<<_Drone_state.velocity[2]<<" [m/s] "<<endl;
 
}

