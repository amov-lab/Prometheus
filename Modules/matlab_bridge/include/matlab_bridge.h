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

enum MATLAB_CMD_X
{
    CHECK = 1,
    TAKEOFF = 2,
    LAND = 3,
    HOLD = 4,
    MATLAB_CMD = 5
};

enum MATLAB_CMD_Y
{
    POS_CTRL_MODE = 1,
    VEL_CTRL_MODE = 2,
    ATT_CTRL_MODE = 3
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
geometry_msgs::Point matlab_setting_cmd;
geometry_msgs::Point matlab_setting_result;
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;


ros::Publisher matlab_setting_result_pub;
ros::Publisher uav_command_pub;
