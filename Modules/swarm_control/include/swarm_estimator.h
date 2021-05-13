#ifndef SWARM_ESTIMATOR_H
#define SWARM_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>

#include <prometheus_msgs/DroneState.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

#include "math_utils.h"
#include "message_utils.h"

// 宏定义
#define NODE_NAME "swarm_estimator"         // 节点名称
#define TRA_WINDOW 10                     // 发布轨迹长度
#define TIMEOUT_MAX 0.1                     // MOCAP超时阈值
// 变量
string uav_name;                            // 无人机名字(话题前缀)
string object_name;                         // 动作捕捉软件中设定的刚体名字
string msg_name;
int input_source;                           // 0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
Eigen::Vector3f pos_offset;                 // 定位设备偏移量
float yaw_offset;                           // 定位设备偏移量
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Vector3d pos_drone_mocap;            // 无人机当前位置 (mocap)
Eigen::Quaterniond q_mocap;                 // 无人机当前姿态 - 四元数 (mocap)
Eigen::Vector3d Euler_mocap;                // 无人机当前姿态 - 欧拉角 (mocap)
ros::Time mocap_timestamp;                  // mocap时间戳
Eigen::Vector3d pos_drone_gazebo;           // 无人机当前位置 (gazebo)
Eigen::Quaterniond q_gazebo;                // 无人机当前姿态 - 四元数 (gazebo)
Eigen::Vector3d Euler_gazebo;               // 无人机当前姿态 - 欧拉角 (gazebo)
prometheus_msgs::Message message;           // 待打印的文字消息
nav_msgs::Odometry Drone_odom;              // 无人机里程计,用于rviz显示
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;    // 无人机轨迹容器,用于rviz显示
// 订阅话题
ros::Subscriber state_sub;
ros::Subscriber extended_state_sub;
ros::Subscriber position_sub;
ros::Subscriber velocity_sub;
ros::Subscriber attitude_sub;
ros::Subscriber alt_sub;
ros::Subscriber mocap_sub;
ros::Subscriber gazebo_sub;
// 发布话题
ros::Publisher drone_state_pub;
ros::Publisher vision_pub;
ros::Publisher message_pub;
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;

void init()
{
    _DroneState.connected = false;
    _DroneState.armed = false;
    _DroneState.mode = "";
    _DroneState.position[0] = 0.0;
    _DroneState.position[1] = 0.0;
    _DroneState.position[2] = 0.0;
    _DroneState.velocity[0] = 0.0;
    _DroneState.velocity[1] = 0.0;
    _DroneState.velocity[2] = 0.0;
    _DroneState.attitude_q.w = 0.0;
    _DroneState.attitude_q.x = 0.0;
    _DroneState.attitude_q.y = 0.0;
    _DroneState.attitude_q.z = 0.0;
    _DroneState.attitude[0] = 0.0;
    _DroneState.attitude[1] = 0.0;
    _DroneState.attitude[2] = 0.0;
    _DroneState.attitude_rate[0] = 0.0;
    _DroneState.attitude_rate[1] = 0.0;
    _DroneState.attitude_rate[2] = 0.0;
    _DroneState.rel_alt = 0.0;
    //
    pos_drone_mocap = Eigen::Vector3d(0.0, 0.0, 0.0);
    q_mocap         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Euler_mocap     = Eigen::Vector3d(0.0, 0.0, 0.0);

    pos_drone_gazebo = Eigen::Vector3d(0.0, 0.0, 0.0);
    q_gazebo         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Euler_gazebo     = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    _DroneState.connected = msg->connected;
    _DroneState.armed = msg->armed;
    _DroneState.mode = msg->mode;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _DroneState.position[0] = msg->pose.position.x;
    _DroneState.position[1] = msg->pose.position.y;
    _DroneState.position[2] = msg->pose.position.z;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    _DroneState.velocity[0] = msg->twist.linear.x;
    _DroneState.velocity[1] = msg->twist.linear.y;
    _DroneState.velocity[2] = msg->twist.linear.z;
}

void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    
    _DroneState.attitude_q.w = q_fcu.w();
    _DroneState.attitude_q.x = q_fcu.x();
    _DroneState.attitude_q.y = q_fcu.y();
    _DroneState.attitude_q.z = q_fcu.z();

    _DroneState.attitude[0] = euler_fcu[0];
    _DroneState.attitude[1] = euler_fcu[1];
    _DroneState.attitude[2] = euler_fcu[2];

    _DroneState.attitude_rate[0] = msg->angular_velocity.x;
    _DroneState.attitude_rate[1] = msg->angular_velocity.x;
    _DroneState.attitude_rate[2] = msg->angular_velocity.x;
}

void alt_cb(const std_msgs::Float64::ConstPtr &msg)
{
    _DroneState.rel_alt = msg->data;
}

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Euler_mocap = quaternion_to_euler(q_mocap);
    // 记录收到mocap的时间戳
    mocap_timestamp = ros::Time::now();
}
void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world")
    {
        pos_drone_gazebo = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_gazebo = quaternion_to_euler(q_gazebo);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "wrong gazebo ground truth frame id.");
    }
}

void timercb_vision(const ros::TimerEvent &e)
{
    geometry_msgs::PoseStamped vision;

    //mocap
    if (input_source == 0)
    {
        vision.pose.position.x = pos_drone_mocap[0];
        vision.pose.position.y = pos_drone_mocap[1];
        vision.pose.position.z = pos_drone_mocap[2];

        vision.pose.orientation.x = q_mocap.x();
        vision.pose.orientation.y = q_mocap.y();
        vision.pose.orientation.z = q_mocap.z();
        vision.pose.orientation.w = q_mocap.w();
        // 如果长时间未收到mocap数据，则一直给飞控发送旧数据，此处显示timeout
        if( get_time_in_sec(mocap_timestamp) > TIMEOUT_MAX)
        {
            pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_name, "Mocap Timeout.");
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
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Wrong input_source.");
    }

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void timercb_drone_state(const ros::TimerEvent &e)
{
    _DroneState.header.stamp = ros::Time::now();
    drone_state_pub.publish(_DroneState);
}

void timercb_rviz(const ros::TimerEvent &e)
{
    // 发布无人机当前odometry,用于导航及rviz显示
    nav_msgs::Odometry Drone_odom;
    Drone_odom.header.stamp = ros::Time::now();
    Drone_odom.header.frame_id = "world";
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];

    // 导航算法规定 高度不能小于0
    if (Drone_odom.pose.pose.position.z <= 0)
    {
        Drone_odom.pose.pose.position.z = 0.01;
    }

    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];
    odom_pub.publish(Drone_odom);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped drone_pos;
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "world";
    drone_pos.pose.position.x = _DroneState.position[0];
    drone_pos.pose.position.y = _DroneState.position[1];
    drone_pos.pose.position.z = _DroneState.position[2];

    drone_pos.pose.orientation = _DroneState.attitude_q;

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

#endif