//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "printf_utils.h"

using namespace std;

#define MATLAB_TIMEOUT 0.2
#define RETURN_INIT_POS_TIMEOUT 60
#define LAND_TIMEOUT 300

enum MATLAB_CMD_X
{
    CHECK = 1,      // 起飞前检查
    TAKEOFF = 2,
    LAND = 3,
    HOLD = 4,
    MATLAB_CMD = 5  // 指令控制
};

enum MATLAB_CMD_Y
{
    POS_CTRL_MODE = 1,  // 位置点控制
    VEL_CTRL_MODE = 2,  // 速度控制
    ATT_CTRL_MODE = 3   // 姿态控制
};

enum MATLAB_RESULT_X
{
    REJECT = 1,
    SUCCESS = 2
};

int uav_id;
int matlab_control_mode;
bool ready_for_matlab_check;
bool ready_for_matlab_cmd;
geometry_msgs::Pose matlab_cmd;
ros::Time last_matlab_cmd_time;
geometry_msgs::Point matlab_setting_cmd;
ros::Time last_matlab_setting_cmd_time;
geometry_msgs::Point matlab_setting_result;
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;

bool cmd_timeout{false};

ros::Publisher matlab_setting_result_pub;
ros::Publisher uav_command_pub;
