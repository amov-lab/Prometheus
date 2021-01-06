/***************************************************************************************************************************
* state_from_mavros.h
*
* Author: Qyp
*
* Update Time: 2020.1.18
*
* 主要功能：
*    本库函数主要用于连接prometheus_control与mavros两个功能包。
*           1、订阅mavros功能包发布的飞控状态量。状态量包括无人机状态、位置、速度、角度、角速度。
*           2、发布无人机运动轨迹，话题为/prometheus/drone_trajectory，可通过参数pos_estimator/state_fromposehistory_window来设置轨迹的长短
*               注1： 这里并没有订阅所有可以来自飞控的消息，如需其他消息，请参阅mavros代码。
*               注2：代码中，参与运算的角度均是以rad为单位，但是涉及到显示时或者需要手动输入时均以deg为单位。
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
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

using namespace std;

class state_from_mavros
{
    public:
    //constructed function
    state_from_mavros(void):
        state_nh("~")
    {
        state_nh.param<string>("uav_name", uav_name, "/uav0");

        if (uav_name == "/uav0")
        {
            uav_name = "";
        }

        // 【订阅】无人机当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
        state_sub = state_nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, &state_from_mavros::state_cb,this);

        // 【订阅】无人机当前状态 - 来自飞控
        extended_state_sub = state_nh.subscribe<mavros_msgs::ExtendedState>(uav_name + "/mavros/extended_state", 10, &state_from_mavros::extended_state_cb,this);

        // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        position_sub = state_nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, &state_from_mavros::pos_cb,this);

        // 【订阅】无人机当前速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        velocity_sub = state_nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, &state_from_mavros::vel_cb,this);

        // 【订阅】无人机当前欧拉角 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
        attitude_sub = state_nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, &state_from_mavros::att_cb,this); 

        // 【订阅】无人机相对高度 此订阅仅针对户外实验
        alt_sub = state_nh.subscribe<std_msgs::Float64>(uav_name + "/mavros/global_position/rel_alt", 10, &state_from_mavros::alt_cb,this);
    }

    //变量声明 
    prometheus_msgs::DroneState _DroneState;
    string uav_name;
    
    private:

        ros::NodeHandle state_nh;

        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber attitude_sub, alt_sub;
        ros::Subscriber extended_state_sub;
        ros::Publisher trajectory_pub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            _DroneState.connected = msg->connected;
            _DroneState.armed = msg->armed;
            _DroneState.mode = msg->mode;
        }

        void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
        {
            if(msg->landed_state == msg->LANDED_STATE_ON_GROUND)
            {
                _DroneState.landed = true;
            }else
            {
                _DroneState.landed = false;
            }
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

};

    
#endif
