/***************************************************************************************************************************
* px4_sender.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  PX4 command sender using px4 default command
*         1. Subscribe command.msg from upper nodes
*         2. Send command using command_to_mavros.h
*         3. Command includes:  (1)xyz+yaw
*                               (2)takeoff
*                               (3)land
*                               (4)idle
*                               (5)loiter
*                               (6)xyz+yaw(body frame)
***************************************************************************************************************************/

#include <ros/ros.h>

#include "state_from_mavros.h"
#include "command_to_mavros.h"
#include "prometheus_control_utils.h"
#include "message_utils.h"


#define NODE_NAME "px4_sender"

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float cur_time;                                             //程序运行时间

//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

prometheus_msgs::DroneState _DroneState;                         //无人机状态量

prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令


Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
double yaw_sp;

float Takeoff_height;
float Disarm_height;
float Use_mocap_raw;

prometheus_msgs::Message message;
prometheus_msgs::LogMessage LogMessage;
ros::Publisher message_pub;
ros::Publisher log_message_pub;
//变量声明 - 其他变量
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();
int check_failsafe();
void Command_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    // CommandID必须递增才会被记录
    if( msg->Command_ID  >  Command_Now.Command_ID )
    {
        Command_Now = *msg;
    }else
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Wrong Command ID.");
    }
    
    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == prometheus_msgs::ControlCommand::Disarm)
    {
        Command_Now = Command_Last;
    }
}
void station_command_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Get a command from Prometheus Station.");
    
    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == prometheus_msgs::ControlCommand::Disarm)
    {
        Command_Now = Command_Last;
    }
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    _DroneState.time_from_start = cur_time;
}
void timerCallback(const ros::TimerEvent& e)
{
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Program is running.");
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_sender");
    ros::NodeHandle nh("~");

    //【订阅】指令
    // 本话题为任务模块生成的控制指令
    ros::Subscriber Command_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, Command_cb);

    //【订阅】指令
    // 本话题为Prometheus地面站发送的控制指令
    ros::Subscriber station_command_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/control_command_station", 10, station_command_cb);
    
    //【订阅】无人机状态
    // 本话题来自px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    // 【发布】用于log的消息
    log_message_pub = nh.advertise<prometheus_msgs::LogMessage>("/prometheus/topic_for_log", 10);

    // 10秒定时打印，以确保程序在正确运行
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timerCallback);
    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);

    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);

    printf_param();
    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;

    // int check_flag;
    // // 这一步是为了程序运行前检查一下参数是否正确
    // // 输入1,继续，其他，退出程序
    // cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    // cin >> check_flag;

    // if(check_flag != 1)
    // {
    //     return -1;
    // }

    // 先读取一些飞控的数据
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();

    }

    
    Eigen::Vector3d Takeoff_position;
    Takeoff_position[0] = _DroneState.position[0];
    Takeoff_position[1] = _DroneState.position[1];
    Takeoff_position[2] = _DroneState.position[2];


    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;


    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = prometheus_control_utils::get_time_in_sec(begin_time);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        cur_time = prometheus_control_utils::get_time_in_sec(begin_time);

        last_time = cur_time;

        ros::spinOnce();

        // Check for geo fence: If drone is out of the geo fence, it will land now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
        }

        // 无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.Mode == prometheus_msgs::ControlCommand::Land)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
        }

        switch (Command_Now.Mode)
        {
        case prometheus_msgs::ControlCommand::Idle:
            _command_to_mavros.idle();

            // 设定yaw_ref=999时，切换offboard模式，并解锁
            if(Command_Now.Reference_State.yaw_ref == 999)
            {
                if(_DroneState.mode != "OFFBOARD")
                {
                    _command_to_mavros.mode_cmd.request.custom_mode = "OFFBOARD";
                    _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Setting to OFFBOARD Mode...");
                    // //执行回调函数
                    // ros::spinOnce();
                    // ros::Duration(0.5).sleep();
                }

                if(!_DroneState.armed)
                {
                    _command_to_mavros.arm_cmd.request.value = true;
                    _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Arming...");
                    // //执行回调函数
                    // ros::spinOnce();
                    // ros::Duration(0.5).sleep();
                }
            }
            break;
            
        case prometheus_msgs::ControlCommand::Takeoff:

            pos_sp = Eigen::Vector3d(Takeoff_position[0],Takeoff_position[1],Takeoff_position[2]+Takeoff_height);
            vel_sp = Eigen::Vector3d(0.0,0.0,0.0);
            yaw_sp = _DroneState.attitude[2]; //rad

            _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);

            break;

        // 【Hold】 悬停。当前位置悬停
        case prometheus_msgs::ControlCommand::Hold:

            if (Command_Last.Mode != prometheus_msgs::ControlCommand::Hold)
            {
                Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
                Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_Now.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_Now.Reference_State.position_ref[2] = _DroneState.position[2];
                Command_Now.Reference_State.yaw_ref         = _DroneState.attitude[2]; //rad

                pos_sp = Eigen::Vector3d(_DroneState.position[0],_DroneState.position[1],_DroneState.position[2]);
                vel_sp = Eigen::Vector3d(0.0,0.0,0.0);
                yaw_sp = _DroneState.attitude[2]; //rad
               
            }
            _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);

            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case prometheus_msgs::ControlCommand::Land:

            if (Command_Last.Mode != prometheus_msgs::ControlCommand::Land)
            {
                pos_sp = Eigen::Vector3d(_DroneState.position[0],_DroneState.position[1],Takeoff_position[2]);
                yaw_sp = _DroneState.attitude[2];
                _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
            }

            //如果距离起飞高度小于10厘米，则直接切换为land模式；
            if(abs(_DroneState.position[2] - Takeoff_position[2]) < Disarm_height)
            {
                if(_DroneState.mode == "OFFBOARD")
                {
                    //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,直接使用飞控中的land模式
                    _command_to_mavros.mode_cmd.request.custom_mode = "AUTO.LAND";
                    _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "LAND: inter AUTO LAND filght mode");
                }
            }

            if(_DroneState.landed)
            {
                Command_Now.Mode = prometheus_msgs::ControlCommand::Idle;
            }

            break;

        case prometheus_msgs::ControlCommand::Move:

            //只有在comid增加时才会进入解算 ： 机体系 至 惯性系
            if( Command_Now.Command_ID  >  Command_Last.Command_ID )
            {
                if( Command_Now.Reference_State.Move_mode  == 0 )
                {
                    pos_sp = Eigen::Vector3d(Command_Now.Reference_State.position_ref[0],Command_Now.Reference_State.position_ref[1],Command_Now.Reference_State.position_ref[2]);
                    yaw_sp = Command_Now.Reference_State.yaw_ref;
                }
            }
            _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
            break;


        // 【Disarm】 上锁
        case prometheus_msgs::ControlCommand::Disarm:

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Disarm: switch to MANUAL flight mode");
            if(_DroneState.mode == "OFFBOARD")
            {
                _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
            }

            if(_DroneState.armed)
            {
                _command_to_mavros.arm_cmd.request.value = false;
                _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
            }
            
            break;

        // 【User_Mode1】 暂空。可进行自定义
        case prometheus_msgs::ControlCommand::User_Mode1:
            
            break;

        // 【User_Mode2】 暂空。可进行自定义
        case prometheus_msgs::ControlCommand::User_Mode2:
            
            break;
        }



        //发布log消息，可用rosbag记录
        LogMessage.time = cur_time;
        LogMessage.Drone_State = _DroneState;
        LogMessage.Control_Command = Command_Now;

        log_message_pub.publish(LogMessage);

        Command_Last = Command_Now;
        rate.sleep();
    }

    return 0;

}


void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> px4_pos_controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height   : "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height    : "<< Disarm_height <<" [m] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;

}

int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        pub_message(message_pub, prometheus_msgs::Message::ERROR, NODE_NAME, "Out of the geo fence, the drone is landing...");
        return 1;
    }
    else{
        return 0;
    }
}

// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}