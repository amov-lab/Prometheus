/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
*
* Update Time: 2020.08.10
*
* Introduction:  PX4 Position Controller 
*         1. 从应用层节点订阅/prometheus/control_command话题（ControlCommand.msg），接收来自上层的控制指令。
*         2. 从px4_pos_estimator.cpp节点订阅无人机的状态信息（DroneState.msg）。
*         3. 调用位置环控制算法，计算加速度控制量，并转换为期望角度。（可选择cascade_PID, PID, UDE, passivity-UDE, NE+UDE位置控制算法）
*         4. 通过command_to_mavros.h将计算出来的控制指令发送至飞控（通过mavros包发送mavlink消息）
*         5. PX4固件通过mavlink_receiver.cpp接收该mavlink消息。
***************************************************************************************************************************/

#include <ros/ros.h>
#include "state_from_mavros.h"
#include "command_to_mavros.h"
#include "prometheus_control_utils.h"
#include "message_utils.h"
#include "Position_Controller/pos_controller_cascade_PID.h"
#include "Position_Controller/pos_controller_PID.h"
#include "Position_Controller/pos_controller_Passivity.h"
#include "Filter/LowPassFilter.h"
#define NODE_NAME "pos_controller"

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float cur_time;                                             //程序运行时间
string controller_type;                                      //控制器类型
float Takeoff_height;                                       //默认起飞高度
float Disarm_height;                                        //自动上锁高度
float Land_speed;                                           //降落速度
//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position;                              // 起飞位置
prometheus_msgs::DroneState _DroneState;                          //无人机状态量

prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令

prometheus_msgs::ControlOutput _ControlOutput;
prometheus_msgs::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
prometheus_msgs::Message message;
prometheus_msgs::LogMessageControl LogMessage;

//RVIZ显示：期望位置
geometry_msgs::PoseStamped ref_pose_rviz;
float dt = 0;

ros::Publisher att_ref_pub;
ros::Publisher rivz_ref_pose_pub;
ros::Publisher message_pub;
ros::Publisher log_message_pub;
Eigen::Vector3d throttle_sp;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int check_failsafe();
void printf_param();
void Body_to_ENU();
void add_disturbance();
geometry_msgs::PoseStamped get_ref_pose_rviz(const prometheus_msgs::ControlCommand& cmd, const prometheus_msgs::AttitudeReference& att_ref);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
    ros::init(argc, argv, "px4_pos_controller");
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

    //【发布】位置控制器的输出量:期望姿态
    att_ref_pub = nh.advertise<prometheus_msgs::AttitudeReference>("/prometheus/control/attitude_reference", 10);      
        
    //【发布】参考位姿 RVIZ显示用
    rivz_ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/control/ref_pose_rviz", 10);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    // 【发布】用于log的消息
    log_message_pub = nh.advertise<prometheus_msgs::LogMessageControl>("/prometheus/log/control", 10);

    // 10秒定时打印，以确保程序在正确运行
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timerCallback);

    // 参数读取
    nh.param<string>("controller_type", controller_type, "default");
    nh.param<float>("Takeoff_height", Takeoff_height, 1.5);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Land_speed", Land_speed, 0.2);
    
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);

    // 位置控制一般选取为50Hz，主要取决于位置状态的更新频率
    ros::Rate rate(50.0);

    float time_trajectory = 0.0;

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;

    // 位置控制器声明
    pos_controller_cascade_PID pos_controller_cascade_pid;
    // 可以设置自定义位置环控制算法
    pos_controller_PID pos_controller_pid;
    pos_controller_passivity pos_controller_passivity;

    printf_param();

    if(controller_type == "default")
    {
        pos_controller_cascade_pid.printf_param();
        
    }else if(controller_type == "pid")
    {
        pos_controller_pid.printf_param();
    }else if(controller_type == "passivity")
    {
        pos_controller_passivity.printf_param();
    }

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
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        //执行回调函数
        ros::spinOnce();

        // Check for geo fence: If drone is out of the geo fence, it will land now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
        }

        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
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
                }else
                {
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "The Drone is in OFFBOARD Mode already...");
                }
                

                if(!_DroneState.armed)
                {
                    _command_to_mavros.arm_cmd.request.value = true;
                    _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Arming...");
                }else
                {
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "The Drone is armd already...");
                }
                
            }
        
            break;

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
        case prometheus_msgs::ControlCommand::Takeoff:
            
            //当无人机在空中时若受到起飞指令，则发出警告并悬停
            // if (_DroneState.landed == false)
            // {
            //     Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            //     pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "The drone is in the air!");
            // }

            if (_DroneState.landed == true && Command_Last.Mode != prometheus_msgs::ControlCommand::Takeoff)
            {
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Takeoff to the desired point.");
                // 设定起飞位置
                Takeoff_position[0] = _DroneState.position[0];
                Takeoff_position[1] = _DroneState.position[1];
                Takeoff_position[2] = _DroneState.position[2];

                //
                Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
                Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.position_ref[0] = Takeoff_position[0];
                Command_Now.Reference_State.position_ref[1] = Takeoff_position[1];
                Command_Now.Reference_State.position_ref[2] = Takeoff_position[2] + Takeoff_height;
                Command_Now.Reference_State.yaw_ref         = _DroneState.attitude[2];
            }
            
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
            }

            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case prometheus_msgs::ControlCommand::Land:

            if (Command_Last.Mode != prometheus_msgs::ControlCommand::Land)
            {
                Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XY_POS_Z_VEL;
                Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_Now.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_Now.Reference_State.velocity_ref[2] = - Land_speed; //Land_speed
                Command_Now.Reference_State.yaw_ref         = _DroneState.attitude[2]; //rad
            }

            //如果距离起飞高度小于10厘米，则直接切换为land模式；
            if(abs(_DroneState.position[2] - Takeoff_position[2]) < Disarm_height)
            {
                if(_DroneState.mode != "AUTO.LAND")
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


        // 【Move】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
        case prometheus_msgs::ControlCommand::Move:

            //对于机体系的指令,需要转换成ENU坐标系执行,且同一ID号内,只执行一次.
            if(Command_Now.Reference_State.Move_frame != prometheus_msgs::PositionReference::ENU_FRAME && Command_Now.Command_ID  >  Command_Last.Command_ID )
            {
                Body_to_ENU();
            }

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

        //执行控制
        if(Command_Now.Mode != prometheus_msgs::ControlCommand::Idle)
        {
            //选择控制器
            if(controller_type == "default")
            {
                //轨迹追踪控制,直接改为PID控制器
                if(Command_Now.Reference_State.Move_mode != prometheus_msgs::PositionReference::TRAJECTORY)
                {
                    _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_Now.Reference_State, dt);
                }else
                {
                    _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_Now.Reference_State, dt);
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "CPID NOT SUPPOORT TRAJECTORY TRACKING.");
                }
                
            }else if(controller_type == "pid")
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }else if(controller_type == "passivity")
            {
                _ControlOutput = pos_controller_passivity.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }
            
        }

        throttle_sp[0] = _ControlOutput.Throttle[0];
        throttle_sp[1] = _ControlOutput.Throttle[1];
        throttle_sp[2] = _ControlOutput.Throttle[2];

        _AttitudeReference = prometheus_control_utils::ThrottleToAttitude(throttle_sp, Command_Now.Reference_State.yaw_ref);

        //发送解算得到的期望姿态角至PX4
        _command_to_mavros.send_attitude_setpoint(_AttitudeReference); 

        //发布期望姿态
        att_ref_pub.publish(_AttitudeReference);

        //发布用于RVIZ显示的位姿
        ref_pose_rviz = get_ref_pose_rviz(Command_Now, _AttitudeReference);   
        rivz_ref_pose_pub.publish(ref_pose_rviz);

        //发布log消息，可用rosbag记录
        LogMessage.control_type = 1;
        LogMessage.time = cur_time;
        LogMessage.Drone_State = _DroneState;
        LogMessage.Control_Command = Command_Now;
        LogMessage.Control_Output = _ControlOutput;
        LogMessage.Attitude_Reference = _AttitudeReference;
        LogMessage.ref_pose = ref_pose_rviz;
        log_message_pub.publish(LogMessage);

        Command_Last = Command_Now;
        rate.sleep();
    }

    return 0;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> px4_pos_controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "controller_type: "<< controller_type <<endl;
    cout << "Takeoff_height   : "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height    : "<< Disarm_height <<" [m] "<<endl;
    cout << "Land_speed       : "<< Land_speed <<" [m/s] "<<endl;
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


//【Body_to_ENU】 机体系移动。
void Body_to_ENU()
{
    
    if( Command_Now.Reference_State.Move_mode  & 0b10 )
    {
        //xy velocity mode
        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame

        //根据无人机当前偏航角进行坐标系转换
        prometheus_control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;
        Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
    }
    else
    {   
        //xy position mode
        float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
        float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
        prometheus_control_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);

        Command_Now.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
        Command_Now.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];
        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
    }

    if(Command_Now.Reference_State.Move_frame == prometheus_msgs::PositionReference::MIX_FRAME)
    {
        Command_Now.Reference_State.position_ref[2] = Command_Now.Reference_State.position_ref[2];
        //Command_Now.Reference_State.yaw_ref = Command_Now.Reference_State.yaw_ref;
    }else
    {
        if( Command_Now.Reference_State.Move_mode  & 0b01 )
        {
            //z velocity mode
            Command_Now.Reference_State.position_ref[2] = 0;
            Command_Now.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];
        }
        else
        {   
            //z posiiton mode
            Command_Now.Reference_State.position_ref[2] = _DroneState.position[2] + Command_Now.Reference_State.position_ref[2];
            Command_Now.Reference_State.velocity_ref[2] = 0; 
        }
        
    }

    Command_Now.Reference_State.yaw_ref = _DroneState.attitude[2] + Command_Now.Reference_State.yaw_ref;
    float d_acc_body[2] = {Command_Now.Reference_State.acceleration_ref[0], Command_Now.Reference_State.acceleration_ref[1]};       
    float d_acc_enu[2]; 

    prometheus_control_utils::rotation_yaw(_DroneState.attitude[2], d_acc_body, d_acc_enu);
    Command_Now.Reference_State.acceleration_ref[0] = d_acc_enu[0];
    Command_Now.Reference_State.acceleration_ref[1] = d_acc_enu[1];
    Command_Now.Reference_State.acceleration_ref[2] = Command_Now.Reference_State.acceleration_ref[2];
}

void add_disturbance()
{

}


geometry_msgs::PoseStamped get_ref_pose_rviz(const prometheus_msgs::ControlCommand& cmd, const prometheus_msgs::AttitudeReference& att_ref)
{
    geometry_msgs::PoseStamped ref_pose;

    ref_pose.header.stamp = ros::Time::now();
    // world: 世界系,即gazebo坐标系,参见tf_transform.launch
    ref_pose.header.frame_id = "world";

    if(cmd.Mode == prometheus_msgs::ControlCommand::Idle)
    {
        ref_pose.pose.position.x = _DroneState.position[0];
        ref_pose.pose.position.y = _DroneState.position[1];
        ref_pose.pose.position.z = _DroneState.position[2];
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }else if(cmd.Mode == prometheus_msgs::ControlCommand::Takeoff || cmd.Mode == prometheus_msgs::ControlCommand::Hold)
    {
        ref_pose.pose.position.x = cmd.Reference_State.position_ref[0];
        ref_pose.pose.position.y = cmd.Reference_State.position_ref[1];
        ref_pose.pose.position.z = cmd.Reference_State.position_ref[2];
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }else if(cmd.Mode == prometheus_msgs::ControlCommand::Disarm  || cmd.Mode == prometheus_msgs::ControlCommand::Land )
    {
        ref_pose.pose.position.x = cmd.Reference_State.position_ref[0];
        ref_pose.pose.position.y = cmd.Reference_State.position_ref[1];
        ref_pose.pose.position.z = 0.0;
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }
    else if(cmd.Mode == prometheus_msgs::ControlCommand::Move)
    {
        //xy速度控制
        if( Command_Now.Reference_State.Move_mode  & 0b10 )
        {
            ref_pose.pose.position.x = _DroneState.position[0] + cmd.Reference_State.velocity_ref[0] * dt;
            ref_pose.pose.position.y = _DroneState.position[1] + cmd.Reference_State.velocity_ref[1] * dt;
        }else
        {
            ref_pose.pose.position.x = cmd.Reference_State.position_ref[0];
            ref_pose.pose.position.y = cmd.Reference_State.position_ref[1];
        }

        if(Command_Now.Reference_State.Move_mode  & 0b01 )
        {
            ref_pose.pose.position.z = _DroneState.position[2] + cmd.Reference_State.velocity_ref[2] * dt;
        }else
        {
            ref_pose.pose.position.z = cmd.Reference_State.position_ref[2];
        }

        ref_pose.pose.orientation = att_ref.desired_att_q;
    }else
    {
        ref_pose.pose.position.x = 0.0;
        ref_pose.pose.position.y = 0.0;
        ref_pose.pose.position.z = 0.0;
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }

    return ref_pose;
}
