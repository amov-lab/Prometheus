#include <ros/ros.h>
#include "state_from_mavros.h"
#include "command_to_mavros.h"
#include "prometheus_control_utils.h"
#include "message_utils.h"
#include "Position_Controller/pos_controller_cascade_PID.h"
#include "Position_Controller/pos_controller_PID.h"
#include "Filter/LowPassFilter.h"

#define NODE_NAME "rover_pos_controller"

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float cur_time;                                             //程序运行时间
int controller_number;                                      //所选择控制器编号
float Takeoff_height;                                       //默认起飞高度
float Disarm_height;                                        //自动上锁高度
float Land_speed;                                           //降落速度
//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

prometheus_msgs::DroneState _DroneState;                          //无人机状态量

prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令

prometheus_msgs::ControlOutput _ControlOutput;
prometheus_msgs::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
prometheus_msgs::Message message;
prometheus_msgs::LogMessage LogMessage;

geometry_msgs::PoseStamped ref_pose_rviz;
float dt = 0;

ros::Publisher att_ref_pub;
ros::Publisher ref_pose_pub;
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
    ros::init(argc, argv, "rover_pos_controller");
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
    ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/control/ref_pose_rviz", 10);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    // 【发布】用于log的消息
    log_message_pub = nh.advertise<prometheus_msgs::LogMessage>("/prometheus/topic_for_log", 10);

    // 10秒定时打印，以确保程序在正确运行
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timerCallback);

    // 参数读取
    nh.param<int>("controller_number", controller_number, 0);
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

    printf_param();

    // 先读取一些飞控的数据
    for(int i=0;i<100;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Set the takeoff position
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

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
        case prometheus_msgs::ControlCommand::Takeoff:

            if (Command_Last.Mode != prometheus_msgs::ControlCommand::Takeoff)
            {
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
            if(Command_Now.Mode == prometheus_msgs::ControlCommand::Move && Command_Now.Reference_State.Move_mode == Command_Now.Reference_State.XYZ_POS)
            {
                Eigen::Vector3d pos_sp;
                float yaw_sp;
                pos_sp[0] = Command_Now.Reference_State.position_ref[0];
                pos_sp[1] = Command_Now.Reference_State.position_ref[1];
                pos_sp[2] = Command_Now.Reference_State.position_ref[2];

                yaw_sp = Command_Now.Reference_State.yaw_ref;

                //选择控制器
                _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
            }

            if(Command_Now.Mode == prometheus_msgs::ControlCommand::Move && Command_Now.Reference_State.Move_mode == Command_Now.Reference_State.XYZ_VEL)
            {
                Eigen::Vector3d vel_sp;
                float yaw_sp;
                vel_sp[0] = Command_Now.Reference_State.velocity_ref[0];
                vel_sp[1] = Command_Now.Reference_State.velocity_ref[1];
                vel_sp[2] = Command_Now.Reference_State.velocity_ref[2];

                yaw_sp = Command_Now.Reference_State.yaw_ref;

                //选择控制器
                _command_to_mavros.send_vel_setpoint(vel_sp, yaw_sp);
            }

            //　目前只有姿态控制还算比较好用
            //　不清楚问题出在哪里：　可能是rover的控制，也可能是混控
            //　注意选择ＦＩｒｍｗａｒｅ中的混控文件　３个可选
            if(Command_Now.Mode == prometheus_msgs::ControlCommand::Move && Command_Now.Reference_State.Move_mode == Command_Now.Reference_State.TRAJECTORY)
            {
                prometheus_msgs::AttitudeReference att_sp;
                float yaw_sp;

                Eigen::Vector3d thr_sp = Eigen::Vector3d(Command_Now.Reference_State.acceleration_ref[0],Command_Now.Reference_State.acceleration_ref[1], 0);
                Eigen::Vector3d euler_sp = Eigen::Vector3d(0.0, 0.0, Command_Now.Reference_State.yaw_ref);

                Eigen::Quaterniond q_sp = quaternion_from_rpy(euler_sp);

                att_sp.throttle_sp[0] = Command_Now.Reference_State.acceleration_ref[0];
                att_sp.throttle_sp[1] = Command_Now.Reference_State.acceleration_ref[1];
                att_sp.throttle_sp[2] = 0.0;

                //期望油门
                double thr_sp_length = thr_sp.norm();
                //　直接赋值
                att_sp.desired_throttle = Command_Now.Reference_State.acceleration_ref[0];

                att_sp.desired_att_q.w = q_sp.w();
                att_sp.desired_att_q.x = q_sp.x();
                att_sp.desired_att_q.y = q_sp.y();
                att_sp.desired_att_q.z = q_sp.z();

                att_sp.desired_attitude[0] = 0.0;  
                att_sp.desired_attitude[1] = 0.0; 
                att_sp.desired_attitude[2] = euler_sp[2]; 

                //选择控制器
                _command_to_mavros.send_attitude_setpoint(att_sp);
            }
        }


        //发布期望姿态
        //att_ref_pub.publish(_AttitudeReference);

        //发布用于RVIZ显示的位姿
        //ref_pose_rviz = get_ref_pose_rviz(Command_Now, _AttitudeReference);   
        //ref_pose_pub.publish(ref_pose_rviz);

        //发布log消息，可用rosbag记录

        //发送解算得到的期望姿态角至PX4
        LogMessage.time = cur_time;
        LogMessage.Drone_State = _DroneState;
        LogMessage.Control_Command = Command_Now;
        //LogMessage.Control_Output = _ControlOutput;
        //LogMessage.Attitude_Reference = _AttitudeReference;
        log_message_pub.publish(LogMessage);

        Command_Last = Command_Now;
        rate.sleep();
    }

    return 0;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> px4_pos_controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "controller_number: "<< controller_number <<endl;
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
