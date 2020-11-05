/***************************************************************************************************************************
 * swarm_estimator.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.10.18
 *
***************************************************************************************************************************/

//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include "state_from_mavros.h"
#include "math_utils.h"
#include "prometheus_control_utils.h"
#include "message_utils.h"

using namespace std;
#define TRA_WINDOW 1000
#define TIMEOUT_MAX 0.05
#define NODE_NAME "swarm_estimator"
//---------------------------------------相关参数-----------------------------------------------
int input_source; //0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
float rate_hz;
Eigen::Vector3f pos_offset;
float yaw_offset;
string uav_name,object_name;
//---------------------------------------vicon定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap; //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap; //无人机当前姿态 (vicon)
//---------------------------------------gazebo真值相关------------------------------------------
Eigen::Vector3d pos_drone_gazebo;
Eigen::Quaterniond q_gazebo;
Eigen::Vector3d Euler_gazebo;
//---------------------------------------发布相关变量--------------------------------------------
ros::Publisher vision_pub;
ros::Publisher drone_state_pub;
ros::Publisher message_pub;
prometheus_msgs::Message message;
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;
prometheus_msgs::DroneState Drone_State;
nav_msgs::Odometry Drone_odom;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
ros::Time last_timestamp;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void send_to_fcu();
void pub_to_nodes(prometheus_msgs::DroneState State_from_fcu);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //位置 -- optitrack系 到 ENU系
    //Frame convention 0: Z-up -- 1: Y-up (See the configuration in the motive software)
    int optitrack_frame = 0;
    if (optitrack_frame == 0)
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
    else
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(-msg->pose.position.x, msg->pose.position.z, msg->pose.position.y);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
    }

    // Transform the Quaternion to Euler Angles
    Euler_mocap = quaternion_to_euler(q_mocap);
    last_timestamp = msg->header.stamp;
}

void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world")
    {
        pos_drone_gazebo = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // pos_drone_gazebo[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_gazebo[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_gazebo[2] = msg->pose.pose.position.z + pos_offset[2];

        q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_gazebo = quaternion_to_euler(q_gazebo);
        // Euler_gazebo[2] = Euler_gazebo[2] + yaw_offset;
        // q_gazebo = quaternion_from_rpy(Euler_gazebo);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "wrong gazebo ground truth frame id.");
    }
}

void timerCallback(const ros::TimerEvent &e)
{
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Program is running.");
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_estimator");
    ros::NodeHandle nh("~");

    //读取参数表中的参数
    nh.param<string>("uav_name", uav_name, "/uav0");
    // 动作捕捉设备中设定的刚体名字
    nh.param<string>("object_name", object_name, "UAV");
    //　程序执行频率
    nh.param<float>("rate_hz", rate_hz, 20);
    // 定位数据输入源 0 for vicon， 1 for 激光SLAM, 2 for gazebo ground truth, 3 for T265
    nh.param<int>("input_source", input_source, 0);
    //　定位设备偏移量
    nh.param<float>("offset_x", pos_offset[0], 0);
    nh.param<float>("offset_y", pos_offset[1], 0);
    nh.param<float>("offset_z", pos_offset[2], 0);
    nh.param<float>("offset_yaw", yaw_offset, 0);

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 1000, mocap_cb);

    // 【订阅】gazebo仿真真值
    ros::Subscriber gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth/p300_basic", 100, gazebo_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 100);

    drone_state_pub = nh.advertise<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10);

    //【发布】无人机odometry
    odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/drone_odom", 10);

    trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/drone_trajectory", 10);
    
    // 【发布】提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 10秒定时打印，以确保程序在正确运行
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timerCallback);

    // 用于与mavros通讯的类，通过mavros接收来至飞控的消息【飞控->mavros->本程序】
    state_from_mavros _state_from_mavros;

    // 频率
    ros::Rate rate(rate_hz);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        // 将采集的机载设备的定位信息及偏航角信息发送至飞控，根据参数input_source选择定位信息来源
        send_to_fcu();

        // 发布无人机状态至其他节点，如px4_pos_controller.cpp节点
        pub_to_nodes(_state_from_mavros._DroneState);

        rate.sleep();
    }

    return 0;
}

void send_to_fcu()
{
    geometry_msgs::PoseStamped vision;

    //vicon
    if (input_source == 0)
    {
        vision.pose.position.x = pos_drone_mocap[0];
        vision.pose.position.y = pos_drone_mocap[1];
        vision.pose.position.z = pos_drone_mocap[2];

        vision.pose.orientation.x = q_mocap.x();
        vision.pose.orientation.y = q_mocap.y();
        vision.pose.orientation.z = q_mocap.z();
        vision.pose.orientation.w = q_mocap.w();
        // 此处时间主要用于监测动捕，T265设备是否正常工作
        if( prometheus_control_utils::get_time_in_sec(last_timestamp) > TIMEOUT_MAX)
        {
            pub_message(message_pub, prometheus_msgs::Message::ERROR, NODE_NAME, "Mocap Timeout.");
        }

    }
    else if (input_source == 2)
    {
        vision.pose.position.x = pos_drone_gazebo[0];
        vision.pose.position.y = pos_drone_gazebo[1];
        vision.pose.position.z = pos_drone_gazebo[2];

        vision.pose.orientation.x = q_gazebo.x();
        vision.pose.orientation.y = q_gazebo.y();
        vision.pose.orientation.z = q_gazebo.z();
        vision.pose.orientation.w = q_gazebo.w();
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Wrong input_source.");
    }

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void pub_to_nodes(prometheus_msgs::DroneState State_from_fcu)
{
    // 发布无人机状态，具体内容参见 prometheus_msgs::DroneState
    Drone_State = State_from_fcu;
    Drone_State.header.stamp = ros::Time::now();
    drone_state_pub.publish(Drone_State);

    // 发布无人机当前odometry,用于导航及rviz显示
    nav_msgs::Odometry Drone_odom;
    Drone_odom.header.stamp = ros::Time::now();
    Drone_odom.header.frame_id = "world";
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = Drone_State.position[0];
    Drone_odom.pose.pose.position.y = Drone_State.position[1];
    Drone_odom.pose.pose.position.z = Drone_State.position[2];

    // 导航算法规定 高度不能小于0
    if (Drone_odom.pose.pose.position.z <= 0)
    {
        Drone_odom.pose.pose.position.z = 0.01;
    }

    Drone_odom.pose.pose.orientation = Drone_State.attitude_q;
    Drone_odom.twist.twist.linear.x = Drone_State.velocity[0];
    Drone_odom.twist.twist.linear.y = Drone_State.velocity[1];
    Drone_odom.twist.twist.linear.z = Drone_State.velocity[2];
    odom_pub.publish(Drone_odom);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped drone_pos;
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "world";
    drone_pos.pose.position.x = Drone_State.position[0];
    drone_pos.pose.position.y = Drone_State.position[1];
    drone_pos.pose.position.z = Drone_State.position[2];

    drone_pos.pose.orientation = Drone_State.attitude_q;

    //发布无人机的位姿 和 轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), drone_pos);
    if (posehistory_vector_.size() > TRA_WINDOW)
    {
        posehistory_vector_.pop_back();
    }

    nav_msgs::Path drone_trajectory;
    drone_trajectory.header.stamp = ros::Time::now();
    drone_trajectory.header.frame_id = "world";
    drone_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(drone_trajectory);
}