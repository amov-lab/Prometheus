// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>
#include <GeographicLib/Geocentric.hpp>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/TextInfo.h>
#include <prometheus_msgs/OffsetPose.h>
#include <prometheus_msgs/GPSData.h>
#include <prometheus_msgs/LinktrackNodeframe2.h>
#include <prometheus_msgs/LinktrackNode2.h>
#include <prometheus_msgs/UAVCommand.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandHome.h>

#include "math_utils.h"
#include "printf_utils.h"
#include "fmt_test.h"
#include "rc_input.h"
using namespace std;

//创建无人机相关数据变量
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

prometheus_msgs::UAVCommand uav_command;      // 指令
Eigen::Vector3d pos_des;
Eigen::Vector3d global_pos_des;
Eigen::Vector3d vel_des;
Eigen::Vector3d acc_des;
double yaw_des;
double yaw_rate_des;

void uav_cmd_cb(const prometheus_msgs::UAVCommand::ConstPtr &msg)
{
    uav_command = *msg;

    if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Move)
    {
        //【Move】 移动，移动子模式的区别详见UAVCommand.msg中的说明
        if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
        {
            // 【XYZ_POS】XYZ惯性系定点控制
            pos_des[0] = uav_command.position_ref[0];
            pos_des[1] = uav_command.position_ref[1];
            pos_des[2] = uav_command.position_ref[2];
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = uav_command.yaw_ref;
        }
        else
        {
            cout << RED  << "Wrong command!" << TAIL << endl;
        }
    }
}

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "fmt_cmd_test");
    //创建句柄
    ros::NodeHandle nh;
    ros::Rate rate(50.0);    
    
    // 发布MAVLINK to FMT （包括服务） - 未测试
    // 【服务】解锁/上锁 - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 【服务】修改系统模式 - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // 【服务】紧急上锁服务(KILL) - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    // 【服务】重启PX4飞控 - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    // 【发布】姿态期望值 - SET_ATTITUDE_TARGET ( #82 )
    ros::Publisher fmt_setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    // 【发布】位置/速度/加速度期望值 - SET_POSITION_TARGET_LOCAL_NED ( #84 )
    ros::Publisher fmt_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    // 【发布】经纬度以及高度位置 - SET_POSITION_TARGET_GLOBAL_INT ( #86 )
    ros::Publisher fmt_setpoint_raw_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 1);
    // 【发布】无人机位置和偏航角，传输至PX4_EKF2模块用于位置姿态估计 - VISION_POSITION_ESTIMATE ( #102 )
    ros::Publisher fmt_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

    // 订阅 上层控制指令
    ros::Subscriber uav_command_sub = nh.subscribe<prometheus_msgs::UAVCommand>("/prometheus/command", 10, uav_cmd_cb);

    cout << GREEN << "Waiting 3s...." << TAIL << endl;
    sleep(3.0);

    // 设置自稳模式
    cout << GREEN << "Switch to POSCTL mode...." << TAIL << endl;
    mode_cmd.request.custom_mode = "POSCTL";
    fmt_set_mode_client.call(mode_cmd);
    sleep(1.0);

    // 解锁
    cout << GREEN << "Arming...." << TAIL << endl;
    arm_cmd.request.value = true;
    fmt_arming_client.call(arm_cmd);
    sleep(1.0);

    // 起飞
    cout << GREEN << "Takeoff by AUTO.TAKEOFF mode...." << TAIL << endl;
    mode_cmd.request.custom_mode = "AUTO.TAKEOFF";
    fmt_set_mode_client.call(mode_cmd);
    sleep(1.0);

    // 设置OFFBOARD模式，并悬停于当前点，同时订阅command进行控制
    cout << GREEN << "Switch to OFFBOARD mode...." << TAIL << endl;
    mode_cmd.request.custom_mode = "OFFBOARD";
    fmt_set_mode_client.call(mode_cmd);

    pos_des << 1.0, 2.0, 3.0;

    while(ros::ok())
    {
        //调用一次回调函数
        ros::spinOnce();

        mavros_msgs::PositionTarget pos_setpoint;
        pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_setpoint.position.x = pos_des.x();
        pos_setpoint.position.y = pos_des.y();
        pos_setpoint.position.z = pos_des.z();
        pos_setpoint.yaw = yaw_des;
        fmt_setpoint_raw_local_pub.publish(pos_setpoint);

        rate.sleep();
    }

    return 0;
}
