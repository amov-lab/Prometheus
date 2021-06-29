#ifndef UGV_ESTIMATOR_H
#define UGV_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>

#include <prometheus_msgs/UgvState.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include "math_utils.h"
#include "printf_utils.h"

// 宏定义
#define TRA_WINDOW 20                     // 发布轨迹长度
#define TIMEOUT_MAX 0.1                     // MOCAP超时阈值
// 变量
int ugv_id;
string ugv_name;                            // 无人机名字(话题前缀)
bool sim_mode;
nav_msgs::Odometry ugv_odom;                // 无人机里程计,用于rviz发布
prometheus_msgs::UgvState ugv_state;
ros::Time last_mocap_timestamp;                  // mocap时间戳
geometry_msgs::PoseStamped vision;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;    // 无人机轨迹容器,用于rviz显示

// 订阅话题
ros::Subscriber state_sub;
ros::Subscriber local_position_sub;
ros::Subscriber global_position_sub;
ros::Subscriber local_velocity_sub;
ros::Subscriber attitude_sub;
ros::Subscriber gazebo_odom_sub;
ros::Subscriber mocap_pos_sub;
ros::Subscriber mocap_vel_sub;
// 发布话题
ros::Publisher ugv_state_pub;
ros::Publisher vision_pub;
ros::Publisher ugv_odom_pub;
ros::Publisher trajectory_pub;


void init(ros::NodeHandle &nh)
{
    // 读取参数
    nh.param("ugv_id", ugv_id, 0);
    nh.param("sim_mode", sim_mode, false);

    ugv_name = "/ugv" + std::to_string(ugv_id);
    
    ugv_state.connected = false;
    ugv_state.armed = false;
    ugv_state.mode = "";
    ugv_state.position[0] = 0.0;
    ugv_state.position[1] = 0.0;
    ugv_state.position[2] = 0.0;
    ugv_state.velocity[0] = 0.0;
    ugv_state.velocity[1] = 0.0;
    ugv_state.velocity[2] = 0.0;
    ugv_state.attitude_q.w = 1.0;
    ugv_state.attitude_q.x = 0.0;
    ugv_state.attitude_q.y = 0.0;
    ugv_state.attitude_q.z = 0.0;
    ugv_state.attitude[0] = 0.0;
    ugv_state.attitude[1] = 0.0;
    ugv_state.attitude[2] = 0.0;

    vision.pose.position.x = 0.0;
    vision.pose.position.y = 0.0;
    vision.pose.position.z = 0.0;

    vision.pose.orientation.x = 0.0;
    vision.pose.orientation.y = 0.0;
    vision.pose.orientation.z = 0.0;
    vision.pose.orientation.w = 1.0;

    last_mocap_timestamp = ros::Time::now();
}

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    // ugv_state赋值 - 状态
    ugv_state.connected = msg->connected;
    ugv_state.armed = msg->armed;
    ugv_state.guided = msg->guided;
    ugv_state.mode = msg->mode;
}

void timercb_vision(const ros::TimerEvent &e)
{
   vision.header.stamp = ros::Time::now();
   vision_pub.publish(vision);

    // 如果长时间未收到mocap数据，则一直给飞控发送旧数据，此处显示timeout
    if(!sim_mode && get_time_in_sec(last_mocap_timestamp) > TIMEOUT_MAX)
    {
        cout << RED << "Mocap Timeout." << TAIL <<endl; 
    }
}

void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 将mocap位置以Vision发送至飞控
    vision = *msg;

    // ugv_state赋值 - 位置
    ugv_state.position[0] = msg->pose.position.x;
    ugv_state.position[1] = msg->pose.position.y;
    ugv_state.position[2] = msg->pose.position.z;
    // ugv_state赋值 - 四元数
    ugv_state.attitude_q = msg->pose.orientation;
    // ugv_state赋值 - 欧拉角
    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d euler_mocap = quaternion_to_euler(q_mocap);
    ugv_state.attitude[0] = euler_mocap[0];
    ugv_state.attitude[1] = euler_mocap[1];
    ugv_state.attitude[2] = euler_mocap[2];

    // 记录收到mocap的时间戳
    last_mocap_timestamp = ros::Time::now();
}

void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // ugv_state赋值 - 速度
    ugv_state.velocity[0] = msg->twist.linear.x;
    ugv_state.velocity[1] = msg->twist.linear.y;
    ugv_state.velocity[2] = msg->twist.linear.z;
}

void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ugv_state赋值 - 状态
    ugv_state.connected = true;
    ugv_state.armed = true;
    ugv_state.guided = false;
    ugv_state.mode = "sim_mode";
    // ugv_state赋值 - 位置
    ugv_state.position[0] = msg->pose.pose.position.x;
    ugv_state.position[1] = msg->pose.pose.position.y;
    ugv_state.position[2] = msg->pose.pose.position.z;
    // ugv_state赋值 - 速度
    ugv_state.velocity[0] = msg->twist.twist.linear.x;
    ugv_state.velocity[1] = msg->twist.twist.linear.y;
    ugv_state.velocity[2] = msg->twist.twist.linear.z;
    // ugv_state赋值 - 四元数
    ugv_state.attitude_q = msg->pose.pose.orientation;
    // ugv_state赋值 - 欧拉角
    Eigen::Quaterniond q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d euler_gazebo = quaternion_to_euler(q_gazebo);
    ugv_state.attitude[0] = euler_gazebo[0];
    ugv_state.attitude[1] = euler_gazebo[1];
    ugv_state.attitude[2] = euler_gazebo[2];
}

void timercb_ugv_state(const ros::TimerEvent &e)
{
    ugv_state.header.stamp = ros::Time::now();
    ugv_state_pub.publish(ugv_state);

    // 发布TF用于RVIZ显示
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    
    //  |----坐标系 ID
    tfs.child_frame_id = ugv_name + "/base_link";  //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = ugv_state.position[0];
    tfs.transform.translation.y = ugv_state.position[1];
    tfs.transform.translation.z = ugv_state.position[2]; 
    //  |--------- 四元数设置  
    tfs.transform.rotation.x = ugv_state.attitude_q.x;
    tfs.transform.rotation.y = ugv_state.attitude_q.y;
    tfs.transform.rotation.z = ugv_state.attitude_q.z;
    tfs.transform.rotation.w = ugv_state.attitude_q.w;

    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);
}

void timercb_rviz(const ros::TimerEvent &e)
{
    // 发布无人机当前odometry,用于导航及rviz显示
    ugv_odom.header.stamp = ros::Time::now();
    ugv_odom.header.frame_id = "world";
    ugv_odom.child_frame_id = "base_link";

    ugv_odom.pose.pose.position.x = ugv_state.position[0];
    ugv_odom.pose.pose.position.y = ugv_state.position[1];
    ugv_odom.pose.pose.position.z = ugv_state.position[2];

    ugv_odom.pose.pose.orientation = ugv_state.attitude_q;
    ugv_odom.twist.twist.linear.x = ugv_state.velocity[0];
    ugv_odom.twist.twist.linear.y = ugv_state.velocity[1];
    ugv_odom.twist.twist.linear.z = ugv_state.velocity[2];
    ugv_odom_pub.publish(ugv_odom);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped ugv_pos;
    ugv_pos.header.stamp = ros::Time::now();
    ugv_pos.header.frame_id = "world";
    ugv_pos.pose.position.x = ugv_state.position[0];
    ugv_pos.pose.position.y = ugv_state.position[1];
    ugv_pos.pose.position.z = ugv_state.position[2];

    ugv_pos.pose.orientation = ugv_state.attitude_q;

    //发布无人机的位姿 和 轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), ugv_pos);
    if (posehistory_vector_.size() > TRA_WINDOW)
    {
        posehistory_vector_.pop_back();
    }

    nav_msgs::Path ugv_trajectory;
    ugv_trajectory.header.stamp = ros::Time::now();
    ugv_trajectory.header.frame_id = "world";
    ugv_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(ugv_trajectory);
}


#endif