//头文件
#include <ros/ros.h>

//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "prometheus_station_utils.h"

using namespace std;
//---------------------------------------相关参数-----------------------------------------------
float refresh_time;

prometheus_msgs::DroneState _DroneState;
prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令

//Target pos of the drone [from fcu]
Eigen::Vector3d pos_drone_fcu_target;
//Target vel of the drone [from fcu]
Eigen::Vector3d vel_drone_fcu_target;
//Target accel of the drone [from fcu]
Eigen::Vector3d accel_drone_fcu_target;
Eigen::Quaterniond q_fcu_target;
Eigen::Vector3d euler_fcu_target;
float Thrust_target;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}
void cmd_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}
void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    euler_fcu_target = prometheus_station_utils::quaternion_to_euler(q_fcu_target);

    Thrust_target = - msg->thrust;
}
void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    ros::Subscriber cmd_sub  = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, cmd_cb);
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, att_target_cb);   
    ros::Subscriber position_target_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, pos_target_cb);

    ros::Rate rate(1.0);

    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        //打印
        printf_info();
        rate.sleep();
    }

    return 0;
}

void printf_info()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Ground Station  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 【打印】无人机状态,包括位置,速度等数据信息
    prometheus_station_utils::prinft_drone_state(_DroneState);

    // 【打印】来自上层的控制指令
    prometheus_station_utils::printf_command_control(Command_Now);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg]  "<<euler_fcu_target[1] * 180/M_PI << " [deg]  "<< euler_fcu_target[2] * 180/M_PI<<" [deg]  "<<endl;
    cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl;

    if(Command_Now.Mode == prometheus_msgs::ControlCommand::Move)
    {
        //Only for TRAJECTORY tracking
        if(Command_Now.Reference_State.Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
        {
            cout <<">>>>>>>>>>>>>>>>>>>>>>>> Tracking Error <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

            static Eigen::Vector3d tracking_error;
            tracking_error = prometheus_station_utils::tracking_error(_DroneState, Command_Now);
            cout << "Pos_error [m]: " << tracking_error[0] <<endl;
            cout << "Vel_error [m/s]: " << tracking_error[1] <<endl;
        }
        
    }

}
