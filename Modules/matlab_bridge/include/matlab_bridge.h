#ifndef MATLAB_BRIDGE_H
#define MATLAB_BRIDGE_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/UAVSetup.h>
#include "geometry_utils.h"
#include "printf_utils.h"

using namespace std;

#define MATLAB_CMD_TIMEOUT 0.5
#define UAV_STATE_TIMEOUT 0.1
#define RETURN_INIT_POS_TIMEOUT 60
#define LAND_TIMEOUT 300

class Matlab_Bridge
{
public:
    Matlab_Bridge(ros::NodeHandle &nh);
    void mainloop();

private:
    int uav_id;
    bool no_rc;
    string agent_name;
    int matlab_control_mode;
    int uav_ready;
    bool uav_checked;
    bool get_matlab_control_cmd;
    geometry_msgs::Pose matlab_cmd;
    ros::Time last_matlab_cmd_time;
    geometry_msgs::Point matlab_setting_cmd;
    ros::Time last_matlab_setting_cmd_time;
    ros::Time get_uav_state_stamp;
    geometry_msgs::Point matlab_setting_result;
    prometheus_msgs::UAVCommand uav_command;
    prometheus_msgs::UAVState uav_state;
    prometheus_msgs::UAVControlState uav_control_state;
    prometheus_msgs::UAVSetup uav_setup;
    bool cmd_timeout{false};

    ros::Subscriber matlab_setting_cmd_sub;
    ros::Subscriber matlab_cmd_sub;
    ros::Subscriber uav_state_sub;
    ros::Subscriber uav_contorl_state_sub;
    
    ros::Publisher matlab_setting_result_pub;
    ros::Publisher uav_command_pub;
    ros::Publisher uav_setup_pub;

    ros::Timer timer_matlab_safety_check;
    ros::Timer timer_printf;
    struct Ctrl_Param_Matlab
    {
        float quad_mass;
        float tilt_angle_max;
        float hov_percent;
        Eigen::Vector3d g;
    };
    Ctrl_Param_Matlab ctrl_param;

    enum MATLAB_CMD_Y
    {
        POS_CTRL_MODE = 1, // 位置点控制
        VEL_CTRL_MODE = 2, // 速度控制
        ACC_CTRL_MODE = 3, // 加速度控制
        ATT_CTRL_MODE = 4  // 姿态控制
    };
    enum MATLAB_CMD_X
    {
        CHECK = 1, // 起飞前检查
        TAKEOFF = 2,
        LAND = 3,
        HOLD = 4,
        MATLAB_CMD = 5 // 指令控制
    };

    enum MATLAB_RESULT_X
    {
        REJECT = 1,
        SUCCESS = 2,
        HEARTBEAT = 3
    };

    void matlab_setting_cmd_cb(const geometry_msgs::Point::ConstPtr &msg);
    void matlab_cmd_cb(const geometry_msgs::Pose::ConstPtr &msg);
    void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg);
    void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg);
    Eigen::Vector4d acc_cmd_to_att_cmd(Eigen::Vector3d &acc_cmd, double yaw_cmd);
    int check_for_uav_state();
    void matlab_safety_check(const ros::TimerEvent &e);
    void printf_msgs(const ros::TimerEvent &e);
};

#endif
