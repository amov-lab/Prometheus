#ifndef gimbal_control_vel_VEL_H
#define gimbal_control_vel_VEL_H

// PX4云台控制类
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <math.h>
#include <mavros_msgs/MountControl.h>
#include <mavros_msgs/MountConfigure.h>
#include <geometry_msgs/Quaternion.h>
#include <prometheus_msgs/ArucoInfo.h>
#include "mission_utils.h"



using namespace std;

class gimbal_control_vel
{
    public:
    gimbal_control_vel(void):
        nh("~")
    {
        // 订阅云台当前角度
        gimbal_att_sub = nh.subscribe<geometry_msgs::Quaternion>("/mavros/gimbal_control/orientation", 10, &gimbal_control_vel::gimbal_att_cb,this);

        // 云台控制：本话题要发送至飞控(通过Mavros_extra功能包 /plugins/gimbal_control.cpp发送)
        mount_control_pub = nh.advertise<mavros_msgs::MountControl>( "/mavros/gimbal_control/command", 1);

        // 云台角度初始化
        gimbal_att        = Eigen::Vector3d(0.0,0.0,0.0);
        gimbal_att_last   = Eigen::Vector3d(0.0,0.0,0.0);

        begin_time = ros::Time::now();

        dt_time = 0.0;

        last_time = get_time_in_sec(begin_time);
    }

    // 云台角度
    Eigen::Vector3d gimbal_att;
    // 上一时刻云台角度
    Eigen::Vector3d gimbal_att_last;
    // 估算的云台角速度
    Eigen::Vector3d gimbal_att_rate;

    // 估算云台角速度
    ros::Time begin_time;
    float last_time;
    float dt_time;

    //发送云台控制指令API函数
    void send_mount_control_command(const Eigen::Matrix<double, 6, 1>& gimbal_att_sp);

    Eigen::Vector3d get_gimbal_att();

    Eigen::Vector3d get_gimbal_att_rate();
    
    private:

        ros::NodeHandle nh;

        ros::Subscriber gimbal_att_sub;
        ros::Publisher mount_control_pub;
        ros::ServiceClient gimbal_mode_control;

        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);

        float get_time_in_sec(const ros::Time& begin_time);

        void gimbal_att_cb(const geometry_msgs::Quaternion::ConstPtr& msg)
        {
            Eigen::Quaterniond gimbal_att_quat;

            gimbal_att_quat = Eigen::Quaterniond(msg->w, msg->x, msg->y, msg->z);

            //Transform the Quaternion to euler Angles
            gimbal_att = quaternion_to_euler(gimbal_att_quat);
            
            float cur_time = get_time_in_sec(begin_time);
            dt_time = cur_time  - last_time;
            dt_time = constrain_function2(dt_time, 0.01, 0.03);
            last_time = cur_time;

            gimbal_att_rate = (gimbal_att - gimbal_att_last)/dt_time;

            gimbal_att_last = gimbal_att;
        }
};

void gimbal_control_vel::send_mount_control_command(const Eigen::Matrix<double, 6, 1>& gimbal_att_sp)
{
  mavros_msgs::MountControl mount_setpoint;
  //
  mount_setpoint.header.stamp = ros::Time::now();
  mount_setpoint.header.frame_id = "map";
  mount_setpoint.mode = 2;
  mount_setpoint.roll = gimbal_att_sp[0]; // Gimbal Roll [deg]
  mount_setpoint.pitch = gimbal_att_sp[1]; // Gimbal   Pitch[deg]
  mount_setpoint.yaw = gimbal_att_sp[2]; // Gimbal  Yaw [deg]
  mount_setpoint.altitude = gimbal_att_sp[3];
  mount_setpoint.latitude = gimbal_att_sp[4];
  mount_setpoint.longitude = gimbal_att_sp[5];

  mount_control_pub.publish(mount_setpoint);

}

Eigen::Vector3d gimbal_control_vel::get_gimbal_att_rate()
{
    return gimbal_att_rate;
}

Eigen::Vector3d gimbal_control_vel::get_gimbal_att()
{
    return gimbal_att;
}

Eigen::Vector3d gimbal_control_vel::quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

float gimbal_control_vel::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}


#endif


