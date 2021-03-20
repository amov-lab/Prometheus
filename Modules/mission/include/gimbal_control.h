#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <math.h>
#include <mavros_msgs/MountControl.h>
#include <geometry_msgs/Quaternion.h>
#include "mission_utils.h"

using namespace std;

class gimbal_control
{
    public:
    //constructed function 1
    gimbal_control(void):
        nh("~")
    {
        nh.param<string>("uav_name", uav_name, "/uav0");

        if (uav_name == "/uav0")
        {
            uav_name = "";
        }
         
        // 订阅云台实际角度
        gimbal_att_sub = nh.subscribe<geometry_msgs::Quaternion>("/mavros/mount_control/orientation", 10, &gimbal_control::gimbal_att_cb,this);

        //　本话题要发送至飞控(通过Mavros_extra功能包 /plugins/mount_control.cpp发送)
        mount_control_pub = nh.advertise<mavros_msgs::MountControl>(uav_name + "/mavros/mount_control/command", 1);

        gimbal_att        = Eigen::Vector3d(0.0,0.0,0.0);
        gimbal_att_last   = Eigen::Vector3d(0.0,0.0,0.0);

        begin_time = ros::Time::now();

        dt_time = 0.0;

        last_time = get_time_in_sec(begin_time);

    }

    string uav_name;

    Eigen::Vector3d gimbal_att;
    Eigen::Vector3d gimbal_att_last;
    Eigen::Vector3d gimbal_att_rate;

    ros::Time begin_time;
    float last_time;
    float dt_time;
    


    //发送云台控制指令
    void send_mount_control_command(const Eigen::Vector3d& gimbal_att_sp);

    Eigen::Vector3d get_gimbal_att();

    Eigen::Vector3d get_gimbal_att_rate();
    
    private:

        ros::NodeHandle nh;

        ros::Subscriber gimbal_att_sub;
        ros::Publisher mount_control_pub;

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

void gimbal_control::send_mount_control_command(const Eigen::Vector3d& gimbal_att_sp)
{
  mavros_msgs::MountControl mount_setpoint;
  //
  mount_setpoint.header.stamp = ros::Time::now();
  mount_setpoint.header.frame_id = "map";
  mount_setpoint.mode = 2;
  mount_setpoint.pitch = gimbal_att_sp[0]; // Gimbal Pitch [deg]
  mount_setpoint.roll = gimbal_att_sp[1]; // Gimbal  Roll [deg]
  mount_setpoint.yaw = gimbal_att_sp[2]; // Gimbal  Yaw [deg]

  mount_control_pub.publish(mount_setpoint);

}

Eigen::Vector3d gimbal_control::get_gimbal_att_rate()
{
    return gimbal_att_rate;
}

Eigen::Vector3d gimbal_control::get_gimbal_att()
{
    return gimbal_att;
}

Eigen::Vector3d gimbal_control::quaternion_to_euler(const Eigen::Quaterniond &q)
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

float gimbal_control::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}


#endif


