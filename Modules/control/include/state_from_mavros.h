/***************************************************************************************************************************
* state_from_mavros.h
*
* Author: Qyp
*
* Update Time: 2019.6.29
*
* 主要功能：
*    本库函数主要用于连接prometheus_control与mavros两个功能包。
* 1、订阅mavros功能包发布的飞控状态量。状态量包括无人机状态、位置、速度、角度、角速度。
*     注： 这里并没有订阅所有可以来自飞控的消息，如需其他消息，请参阅mavros代码。
*     注意：代码中，参与运算的角度均是以rad为单位，但是涉及到显示时或者需要手动输入时均以deg为单位。
*
***************************************************************************************************************************/
#ifndef STATE_FROM_MAVROS_H
#define STATE_FROM_MAVROS_H

#include <ros/ros.h>
#include <bitset>
#include <math_utils.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>


using namespace std;

class state_from_mavros
{
    public:
    //constructed function
    state_from_mavros(void):
        state_nh("~")
    {
        // 【订阅】无人机当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
        state_sub = state_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_from_mavros::state_cb,this);

        // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        position_sub = state_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &state_from_mavros::pos_cb,this);

        // 【订阅】无人机当前速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        velocity_sub = state_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &state_from_mavros::vel_cb,this);

        // 【订阅】无人机当前欧拉角 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
        attitude_sub = state_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &state_from_mavros::att_cb,this);

        trajectory_pub = state_nh.advertise<nav_msgs::Path>("/prometheus/drone_trajectory", 10);
        
        drone_trajectory.header.stamp = ros::Time::now();
        drone_trajectory.header.frame_id = "map";
        //geometry_msgs::PoseStamped null_pose;
        //for(int i=0;i<100;i++)
        //{
        //    drone_trajectory.poses.push_back(null_pose);
        //}

    }

    //变量声明 
    prometheus_msgs::DroneState _DroneState;

    private:

        ros::NodeHandle state_nh;

        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber attitude_sub;
        ros::Publisher trajectory_pub;

        nav_msgs::Path drone_trajectory;

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
            // 2020.1.15备注：目前/mavros/local_position/pose消息中也包含了四元数信息，但究其本质还是来自/plugins/imu.cpp，故此处并不复制其四元数
            // 但刚好借用其 发布无人机运动轨迹，用作rviz画图显示
            drone_trajectory.poses.push_back(*msg);

            //geometry_msgs::PoseStamped a;
           // a = drone_trajectory.poses.front();

            //drone_trajectory.poses.erase(a);
            //drone_trajectory.poses.pop_back();

            trajectory_pub.publish(drone_trajectory);
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


};

    
#endif
