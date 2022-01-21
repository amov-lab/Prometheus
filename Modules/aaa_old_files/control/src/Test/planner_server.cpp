//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <quadrotor_msgs/PositionCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/ControlCommand.h>

using namespace std;

#define NODE_NAME "planner_server"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool sim_mode;  // 选择Gazebo仿真模式 或 真实实验模式
bool drone_ok;
int controller_flag;

prometheus_msgs::DroneState _DroneState;    // 无人机状态

quadrotor_msgs::PositionCommand cmd_from_planning;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令

ros::Publisher command_pub;

void planning_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    cmd_from_planning = *msg;

    if(cmd_from_planning.trajectory_flag == 1)
    {

        //flag为1，代表轨迹可用
        Command_Now.header.stamp                                          = ros::Time::now();
        Command_Now.Mode                                                           = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                                          = Command_Now.Command_ID + 1;
        Command_Now.source                                                         = NODE_NAME;

        if(controller_flag == 0)
        {
            // pos_controller
            Command_Now.Reference_State.Move_mode          = prometheus_msgs::PositionReference::TRAJECTORY;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        }
        else if (controller_flag ==1)
        {
            // px4_sender
            Command_Now.Reference_State.Move_mode          = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        }
        Command_Now.Reference_State.position_ref[0]     = cmd_from_planning.position.x;
        Command_Now.Reference_State.position_ref[1]     = cmd_from_planning.position.y;
        Command_Now.Reference_State.position_ref[2]     = cmd_from_planning.position.z;
        Command_Now.Reference_State.velocity_ref[0]     = cmd_from_planning.velocity.x;
        Command_Now.Reference_State.velocity_ref[1]     = cmd_from_planning.velocity.y;
        Command_Now.Reference_State.velocity_ref[2]     = cmd_from_planning.velocity.z;
        Command_Now.Reference_State.acceleration_ref[0]     = cmd_from_planning.acceleration.x;
        Command_Now.Reference_State.acceleration_ref[1]     = cmd_from_planning.acceleration.y;
        Command_Now.Reference_State.acceleration_ref[2]     = cmd_from_planning.acceleration.z;
        Command_Now.Reference_State.yaw_ref                   = cmd_from_planning.yaw;
        Command_Now.Reference_State.yaw_rate_ref         = cmd_from_planning.yaw_dot;

        if(drone_ok)
        {
            command_pub.publish(Command_Now);   
        }
        
    }else
    {
        cout << "wrong  trajectory_flag"<<endl;
    }
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_server");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    //【订阅】规划器 traj_server发布的控制指令
    ros::Subscriber landpad_det_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, planning_cmd_cb);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【发布】转化为prometheus可用的控制指令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // 仿真模式 - 区别在于是否自动切换offboard模式
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<int>("controller_flag", controller_flag, 0);

    

    drone_ok = false;

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


    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;

    if(sim_mode)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Planner Server Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
            cin >> start_flag;
        }

        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.yaw_ref = 999;
            command_pub.publish(Command_Now);   
            cout << "Switch to OFFBOARD and arm ..."<<endl;
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }else
    {
        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            cout << "Waiting for the offboard mode"<<endl;
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
    }

    // 起飞
    cout<<"[planner_server]: "<<"Takeoff to predefined position."<<endl;

    while( _DroneState.position[2] < 0.5)
    {      
        Command_Now.header.stamp                        = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source                              = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();
        ros::spinOnce();
    }

    drone_ok = true;

    // 等待
    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}
