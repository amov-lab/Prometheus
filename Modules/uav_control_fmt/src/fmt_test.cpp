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
prometheus_msgs::UAVState uav_state;
sensor_msgs::Imu imu_raw;
bool uav_state_update{false};
RC_Input rc_input;


// PX4中的位置设定值（用于验证控制指令是否正确发送）
Eigen::Vector3d fmt_pos_target;
// PX4中的速度设定值（用于验证控制指令是否正确发送）
Eigen::Vector3d fmt_vel_target;
// PX4中的加速度设定值（用于验证控制指令是否正确发送）
Eigen::Vector3d fmt_acc_target;
// PX4中的姿态设定值（用于验证控制指令是否正确发送）
Eigen::Vector3d fmt_att_target;
Eigen::Vector3d fmt_rates_target;
// PX4中的推力设定值（用于验证控制指令是否正确发送）
float fmt_thrust_target;

void set_stream_rate(ros::NodeHandle& nh, int msg_id, int msg_rate);
void printf_uav_state(const ros::TimerEvent &event);
void printf_gps_status();
void fmt_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_state.connected = msg->connected;
    uav_state.armed = msg->armed;
    uav_state.mode = msg->mode;
    uav_state_update = true;
}
void fmt_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    uav_state.battery_state = msg->voltage;
    uav_state.battery_percetage = msg->percentage;
}
void fmt_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_state.position[0] = msg->pose.position.x;
    uav_state.position[1] = msg->pose.position.y;
    uav_state.position[2] = msg->pose.position.z;
}
void fmt_global_rel_alt_cb(const std_msgs::Float64::ConstPtr &msg)
{
    uav_state.rel_alt = msg->data;
}
void fmt_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_state.velocity[0] = msg->twist.linear.x;
    uav_state.velocity[1] = msg->twist.linear.y;
    uav_state.velocity[2] = msg->twist.linear.z;
}
void fmt_att_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);

    uav_state.attitude_q = msg->orientation;
    uav_state.attitude[0] = euler_fcu[0];
    uav_state.attitude[1] = euler_fcu[1];
    uav_state.attitude[2] = euler_fcu[2];
    uav_state.attitude_rate[0] = msg->angular_velocity.x;
    uav_state.attitude_rate[1] = msg->angular_velocity.y;
    uav_state.attitude_rate[2] = msg->angular_velocity.z;
}
void fmt_imu_raw_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_raw = *msg;
}
void gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    uav_state.gps_status = msg->fix_type;
}
void fmt_range_cb(const sensor_msgs::Range::ConstPtr &msg)
{
    uav_state.range = msg->range;
}

void fmt_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_state.latitude = msg->latitude;
    uav_state.longitude = msg->longitude;
    uav_state.altitude = msg->altitude;
}

void fmt_rc_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
    // 调用外部函数对遥控器数据进行处理，具体见rc_data.h，此时rc_input中的状态已更新
    rc_input.handle_rc_data(msg);
}

void fmt_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    fmt_pos_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    fmt_vel_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    fmt_acc_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
}

void fmt_att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    Eigen::Quaterniond fmt_q_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    fmt_att_target = quaternion_to_euler(fmt_q_target);
    fmt_rates_target = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);
    fmt_thrust_target = msg->thrust;
}

void set_fmt_mode_func(string mode)
{

}

void arm_disarm_func(bool on_or_off)
{

}
//发送位置期望值至飞控（输入: 期望xyz,期望yaw）
void send_pos_setpoint(const Eigen::Vector3d &pos_sp, float yaw_sp)
{

}

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "fmt_test");
    //创建句柄
    ros::NodeHandle nh;
    ros::Rate rate(50.0);    
    
    // 目前仅参照uav_control模块进行梳理，可能未来还有一些其他需要交换的Mavlink消息
    // 从FMT获取MAVLINK消息 - 摘自uav_estimator.cpp
    // 【订阅】无人机当前状态 - HEARTBEAT #0
    ros::Subscriber fmt_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, fmt_state_cb);
    // 【订阅】无人机电池状态 - SYS_STATUS #1
    ros::Subscriber fmt_battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, fmt_battery_cb);
    // 【订阅】无人机当前位置 - LOCAL_POSITION_NED #32
    ros::Subscriber fmt_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, fmt_pos_cb);
    // 【订阅】无人机当前位置 - LOCAL_POSITION_NED #32
    ros::Subscriber fmt_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, fmt_vel_cb);
    // 【订阅】无人机当前欧拉角 - ATTITUDE #30 or ATTITUDE_QUATERNION #31 (2选1都可)
    ros::Subscriber fmt_attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, fmt_att_cb);
    // 【订阅】无人机定高雷达数据 - DISTANCE_SENSOR #132
    ros::Subscriber fmt_range_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 10, fmt_range_cb);
    // 【订阅】GPS状态 - GPS_RAW_INT #24
    ros::Subscriber gps_status_sub = nh.subscribe<mavros_msgs::GPSRAW>("/mavros/gpsstatus/gps1/raw", 10, gps_status_cb);
    // 【订阅】无人机当前经纬度 - GLOBAL_POSITION_INT #33
    ros::Subscriber fmt_global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, fmt_global_pos_cb);
    // 【订阅】无人机当前真实高度 - GLOBAL_POSITION_INT #33
    ros::Subscriber fmt_rel_alt_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1, fmt_global_rel_alt_cb);
    // 从FMT获取MAVLINK消息 - 摘自uav_controller.cpp
    //【订阅】遥控器信息 - RC_CHANNELS_RAW #35 or RC_CHANNELS #65
    ros::Subscriber fmt_rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, fmt_rc_cb);    
    //【订阅】PX4中无人机的位置/速度/加速度设定值 - POSITION_TARGET_LOCAL_NED #85 
    ros::Subscriber fmt_position_target_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 1, fmt_pos_target_cb);
    //【订阅】PX4中无人机的姿态设定值 - ATTITUDE_TARGET #83
    ros::Subscriber fmt_attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 1, fmt_att_target_cb);
    // 从FMT获取MAVLINK消息 - 下面是我个人认为还需要考虑的MAVLINK消息，但目前不在uav_control中
    // 【订阅】无人机原始imu信息 - HIGHRES_IMU #105
    ros::Subscriber fmt_imu_raw_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 10, fmt_imu_raw_cb);
    
    // 发布MAVLINK to FMT （包括服务） - 未测试
    // 【发布】无人机位置和偏航角，传输至PX4_EKF2模块用于位置姿态估计 - VISION_POSITION_ESTIMATE ( #102 )
    ros::Publisher fmt_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    // 【发布】位置/速度/加速度期望值 - SET_POSITION_TARGET_LOCAL_NED ( #84 )
    ros::Publisher fmt_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    // 【发布】经纬度以及高度位置 - SET_POSITION_TARGET_GLOBAL_INT ( #86 )
    ros::Publisher fmt_setpoint_raw_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 1);
    // 【发布】姿态期望值 - SET_ATTITUDE_TARGET ( #82 )
    ros::Publisher fmt_setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    // 【服务】解锁/上锁 - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 【服务】修改系统模式 - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // 【服务】紧急上锁服务(KILL) - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    // 【服务】重启PX4飞控 - COMMAND_LONG ( #76 )
    ros::ServiceClient fmt_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    // FMT默认只发送HEARTBEAT心跳包，其他所有Mavlink消息需要进行配置
    // HIGHRES_IMU是ID，100是频率
    set_stream_rate(nh, SYS_STATUS, 5);
    set_stream_rate(nh, GPS_RAW_INT, 10);                  // PX4 默认是unlimited_rate
    set_stream_rate(nh, ATTITUDE, 100);
    set_stream_rate(nh, ATTITUDE_QUATERNION, 100);
    set_stream_rate(nh, LOCAL_POSITION_NED, 50);
    set_stream_rate(nh, GLOBAL_POSITION_INT, 50);
    set_stream_rate(nh, RC_CHANNELS, 20);
    set_stream_rate(nh, ATTITUDE_TARGET, 2);
    set_stream_rate(nh, POSITION_TARGET_LOCAL_NED, 2);
    set_stream_rate(nh, HIGHRES_IMU, 50);
    set_stream_rate(nh, DISTANCE_SENSOR, 10);
    // todo 继续添加其他mavlink消息

    cout << GREEN << "Waiting...." << TAIL << endl;
    sleep(10.0);

    ros::Timer mavlink_printf_timer = nh.createTimer(ros::Duration(1.0), printf_uav_state);


    Eigen::Vector3d pos_des;

    pos_des << 1.0,1.0,1.0;

    int CMD = 999;

    while(ros::ok())
    {
        //调用一次回调函数
        ros::spinOnce();

        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Mavlink Send Test<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "Please choose the CMD: 0 for ARM, 1 for OFFBOARD, 2 MOVE, 3 for else..." << TAIL << endl;
        cin >> CMD;

        if(CMD == 0)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            fmt_arming_client.call(arm_cmd);
        }
        else if(CMD == 1)
        {
            mavros_msgs::SetMode mode_cmd;
            mode_cmd.request.custom_mode = "OFFBOARD";
            fmt_set_mode_client.call(mode_cmd);
        }
        else if(CMD == 2)
        {
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
            pos_setpoint.position.x = 2.0;
            pos_setpoint.position.y = 1.0;
            pos_setpoint.position.z = 3.0;
            pos_setpoint.yaw = 0.0;
            fmt_setpoint_raw_local_pub.publish(pos_setpoint);
        }
        rate.sleep();
    }

    return 0;
}

void set_stream_rate(ros::NodeHandle& nh, int msg_id, int msg_rate)
{
    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

    mavros_msgs::StreamRate srv;
    srv.request.stream_id = msg_id;
    srv.request.message_rate = msg_rate;
    srv.request.on_off = true;

    if (stream_rate_client.call(srv)) 
    {
        cout << GREEN << "Set MAVLINK ID [" << msg_id << "] rate to "<< msg_rate << "Hz successfully!" << TAIL << endl;
    } else 
    {
        cout << RED << "Set MAVLINK ID [" << msg_id << "] rate to "<< msg_rate << "Hz failed!" << TAIL << endl;
    }
}

void printf_uav_state(const ros::TimerEvent &event)
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> FMT UAV State  <<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印 无人机状态
    if (uav_state.connected == true)
    {
        cout << GREEN << "FMT Status:  [ Connected ] ";
    }
    else
    {
        cout << RED << "FMT Status:[ Unconnected ] ";
    }
    //是否上锁
    if (uav_state.armed == true)
    {
        cout << GREEN << "[  Armed   ] ";
    }
    else
    {
        cout << RED << "[ DisArmed ] ";
    }

    cout << "[ " << uav_state.mode << " ] " << TAIL << endl;

    cout << GREEN << "Location: [ GPS ] " << TAIL;
    printf_gps_status();

    cout << GREEN << "UAV_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y Z] : " << uav_state.velocity[0] << " [m/s] " << uav_state.velocity[1] << " [m/s] " << uav_state.velocity[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [R P Y] : " << uav_state.attitude[0] * 180 / M_PI << " [deg] " << uav_state.attitude[1] * 180 / M_PI << " [deg] " << uav_state.attitude[2] * 180 / M_PI << " [deg] " << TAIL << endl;
    cout << GREEN << "rel_alt         : " << uav_state.rel_alt<< " [ m ] "<< TAIL << endl;
    cout << GREEN << "distance range  : " << uav_state.range << " [ m ] "<< TAIL << endl;
    cout << GREEN << "Battery Voltage : " << uav_state.battery_state << " [V] " << "  Battery Percent : " << uav_state.battery_percetage << TAIL << endl;


    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> FMT IMU RAW  <<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    // 目前只打印部分作为检查用
    cout << GREEN << "imu_raw.orientation.x : " << imu_raw.orientation.x << " [ m ] "<< TAIL << endl;
    // 打印遥控器信息
    rc_input.printf_info();

    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> FMT CONTROL TARGET  <<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "Pos_target [X Y Z] : " << fmt_pos_target[0] << " [ m ] " << fmt_pos_target[1] << " [ m ] " << fmt_pos_target[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel_target [X Y Z] : " << fmt_vel_target[0] << " [m/s] " << fmt_vel_target[1] << " [m/s] " << fmt_vel_target[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att_target [R P Y] : " << fmt_att_target[0] * 180 / M_PI << " [deg] " << fmt_att_target[1] * 180 / M_PI << " [deg] " << fmt_att_target[2] * 180 / M_PI << " [deg] " << TAIL << endl;
    cout << GREEN << "Thr_target [ 0-1 ] : " << fmt_thrust_target << TAIL << endl;
}


void printf_gps_status()
{
    switch (uav_state.gps_status)
    {
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS:
        cout << RED << " [GPS_FIX_TYPE_NO_GPS] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_FIX:
        cout << RED << " [GPS_FIX_TYPE_NO_FIX] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_2D_FIX:
        cout << YELLOW << " [GPS_FIX_TYPE_2D_FIX] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX:
        cout << GREEN << " [GPS_FIX_TYPE_3D_FIX] " << TAIL << endl;
        break;
    }

    cout << GREEN << "GPS [lat lon alt] : " << uav_state.latitude << " [ deg ] " << uav_state.longitude << " [ deg ] " << uav_state.altitude << " [ m ] " << TAIL << endl;
}