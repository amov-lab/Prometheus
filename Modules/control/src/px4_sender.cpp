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

#include <state_from_mavros.h>
#include <command_to_mavros.h>

#include <pos_controller_PID.h>
#include <pos_controller_UDE.h>
#include <pos_controller_Passivity.h>
#include <pos_controller_cascade_PID.h>
#include <pos_controller_NE.h>
#include <circle_trajectory.h>

#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/TrajectoryPoint.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_control_utils.h>
#include <prometheus_msgs/Trajectory.h>

#include <Eigen/Eigen>

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令

prometheus_msgs::DroneState _DroneState;                         //无人机状态量
Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
double yaw_sp;

float Takeoff_height;
float Disarm_height;
float Use_mocap_raw;

//变量声明 - 其他变量
//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);
float get_time_in_sec(ros::Time begin);
void prinft_command_state();
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
void printf_param();
int check_failsafe();
void Command_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_sender");
    ros::NodeHandle nh("~");

    ros::Subscriber Command_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus_msgs/control_command", 10, Command_cb);

    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Use_mocap_raw", Use_mocap_raw, 0.0);
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);

    printf_param();
    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // 用于与mavros通讯的类，通过mavros接收来至飞控的消息【飞控->mavros->本程序】
    state_from_mavros _state_from_mavros;
    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 先读取一些飞控的数据
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();

    }

    
    // Set the takeoff position
    Takeoff_position[0] = _state_from_mavros._DroneState.position[0];
    Takeoff_position[1] = _state_from_mavros._DroneState.position[1];
    Takeoff_position[2] = _state_from_mavros._DroneState.position[2];


    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode = command_to_mavros::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
    Command_Now.Reference_State.position_ref[0] = 0;
    Command_Now.Reference_State.position_ref[1] = 0;
    Command_Now.Reference_State.position_ref[2] = 0;
    Command_Now.Reference_State.velocity_ref[0] = 0;
    Command_Now.Reference_State.velocity_ref[1] = 0;
    Command_Now.Reference_State.velocity_ref[2] = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref = 0;


    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        float cur_time = get_time_in_sec(begin_time);

        // 获取当前无人机状态
        _DroneState.time_from_start = cur_time;

        _DroneState = _state_from_mavros._DroneState;

        // 打印无人机状态
        prometheus_control_utils::prinft_drone_state(_DroneState);

        //Printf the command state
        prinft_command_state();

        // 无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.Mode == command_to_mavros::Land)
        {
            Command_Now.Mode = command_to_mavros::Land;
        }

        // Check for geo fence: If drone is out of the geo fence, it will land now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = command_to_mavros::Land;
        }

        switch (Command_Now.Mode)
        {
        case command_to_mavros::Idle:
            _command_to_mavros.idle();
            break;
            
        case command_to_mavros::Takeoff:
            pos_sp = Eigen::Vector3d(Takeoff_position[0],Takeoff_position[1],Takeoff_position[2]+Takeoff_height);
            vel_sp = Eigen::Vector3d(0.0,0.0,0.0);
            yaw_sp = _DroneState.attitude[2]; //rad

            _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);

            break;

        // 不支持复合模式
        case command_to_mavros::Move_ENU:

            //只有在comid增加时才会进入解算 ： 机体系 至 惯性系
            if( Command_Now.Command_ID  >  Command_Last.Command_ID )
            {
                if( Command_Now.Reference_State.Sub_mode  == 0 )
                {
                    pos_sp = Eigen::Vector3d(Command_Now.Reference_State.position_ref[0],Command_Now.Reference_State.position_ref[1],Command_Now.Reference_State.position_ref[2]);
                    yaw_sp = Command_Now.Reference_State.yaw_ref;

                    _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
                }
                else if( Command_Now.Reference_State.Sub_mode  == 3 )
                {
                    vel_sp = Eigen::Vector3d(Command_Now.Reference_State.velocity_ref[0],Command_Now.Reference_State.velocity_ref[1],Command_Now.Reference_State.velocity_ref[2]);

                    _command_to_mavros.send_vel_setpoint(vel_sp, yaw_sp);
                }
            }
            break;

        // 不支持复合模式
        case command_to_mavros::Move_Body:

            if( Command_Now.Reference_State.Sub_mode  == 0 )
            {
                float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
                float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
                rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);

                pos_sp[0] = _DroneState.position[0] + d_pos_enu[0];
                pos_sp[1] = _DroneState.position[1] + d_pos_enu[1];
                pos_sp[2] = _DroneState.position[2] + Command_Now.Reference_State.position_ref[2];

                yaw_sp = _DroneState.attitude[2] + Command_Now.Reference_State.yaw_ref;

                _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
            }
            else if( Command_Now.Reference_State.Sub_mode  == 3 )
            {
                vel_sp = Eigen::Vector3d(Command_Now.Reference_State.velocity_ref[0],Command_Now.Reference_State.velocity_ref[1],Command_Now.Reference_State.velocity_ref[2]);

                yaw_sp =  Command_Now.Reference_State.yaw_ref;

                _command_to_mavros.send_vel_setpoint_body(vel_sp, yaw_sp);
            }


            break;

        case command_to_mavros::Hold:
            if (Command_Last.Mode != command_to_mavros::Hold)
            {
                pos_sp = Eigen::Vector3d(_DroneState.position[0],_DroneState.position[1],_DroneState.position[2]);
                yaw_sp = _DroneState.attitude[2];
            }

            _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
            break;


        case command_to_mavros::Land:
            if (Command_Last.Mode != command_to_mavros::Land)
            {
                pos_sp = Eigen::Vector3d(_DroneState.position[0],_DroneState.position[1],Takeoff_position[2]);
                yaw_sp = _DroneState.attitude[2];
            }

            //如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            //如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            if(abs(_DroneState.position[2] - Takeoff_position[2]) < Disarm_height)
            {
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

                if (_command_to_mavros.arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {

                _command_to_mavros.send_pos_setpoint(pos_sp, yaw_sp);
            }

            break;

        case command_to_mavros::Disarm:
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

            if (_command_to_mavros.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        case command_to_mavros::PPN_land:

            break;

        }


        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;

}

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
// 【打印控制指令函数】
void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    switch(Command_Now.Mode)
    {
    case command_to_mavros::Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;
        break;
    case command_to_mavros::Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;
        break;
    case command_to_mavros::Hold:
        cout << "Command: [ Hold ] " <<endl;
        break;
    case command_to_mavros::Land:
        cout << "Command: [ Land ] " <<endl;
        break;
    case command_to_mavros::Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;
    case command_to_mavros::PPN_land:
        cout << "Command: [ PPN_land ] " <<endl;
        break;
    case command_to_mavros::Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;
    case command_to_mavros::Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        break;

    }

    int sub_mode;
    sub_mode = Command_Now.Reference_State.Sub_mode ;

    if((sub_mode & 0b10) == 0) //xy channel
    {
        cout << "Submode: xy position control "<<endl;
        cout << "X_setpoint   : " << Command_Now.Reference_State.position_ref[0] << " [ m ]"  << "  Y_setpoint : "<< Command_Now.Reference_State.position_ref[1] << " [ m ]"<<endl;
    }
    else{
        cout << "Submode: xy velocity control "<<endl;
        cout << "X_setpoint   : " << Command_Now.Reference_State.velocity_ref[0] << " [m/s]" << "  Y_setpoint : "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s]" <<endl;
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        cout << "Submode:  z position control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.Reference_State.position_ref[2] << " [ m ]" << endl;
    }
    else
    {
        cout << "Submode:  z velocity control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s]" <<endl;
    }

    cout << "Yaw_setpoint : "  << Command_Now.Reference_State.yaw_ref * 180/M_PI<< " [deg] " <<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height: "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height : "<< Disarm_height <<" [m] "<<endl;
    cout << "Use_mocap_raw : "<< Use_mocap_raw <<" [1 for use mocap raw data] "<<endl;
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
        return 1;
        cout << "Out of the geo fence, the drone is landing... "<< endl;
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