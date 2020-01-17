/***************************************************************************************************************************
* terminal_control.cpp
*
* Author: Qyp
*
* Update Time: 2020.1.10
*
* Introduction:  test function for sending ControlCommand.msg
***************************************************************************************************************************/
#include <ros/ros.h>
#include <controller_test.h>
#include <iostream>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

prometheus_msgs::ControlCommand Command_Now;
mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void generate_com(int Move_mode, float state_desired[4]);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    // 【订阅】无人机当前状态 - 来自飞控
    //  本话题来自飞控(通过/plugins/sys_status.cpp)
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【服务】修改系统模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    int Control_Mode = 0;
    int Move_mode = 0;
    int Move_frame = 0;
    int Trjectory_mode = 0;
    float trajectory_total_time = 0;
    float state_desired[4];

    // 圆形轨迹追踪类
    Controller_Test Controller_Test;
    Controller_Test.printf_param();

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


    while(ros::ok())
    {
        // Waiting for input
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Control Test<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Input the Mode: 0 for Idle, 1 for Takeoff, 2 for Hold, 3 for Land, 4 for Move, 5 for Disarm, 6 for User_Mode1, 7 for User_Mode2"<<endl;
        cout << "Input 999 to switch to offboard mode and arm the drone"<<endl;
        cin >> Control_Mode;

        if(Control_Mode == prometheus_msgs::ControlCommand::Move)
        {
            cout << "Input the Move_mode: 0 for xy/z position control, 3 for xy/z velocity control, 4 for trajectory tracking"<<endl;
            cin >> Move_mode;

            if(Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
            {
                cout << "Input the Trajectory: 0 for Circle, 1 for Eight Shape, 2 for Step"<<endl;
                cin >> Trjectory_mode;  
                cout << "Input the trajectory_total_time:"<<endl;
                cin >> trajectory_total_time;
            }else
            {
                cout << "Input the Move_frame: 0 for ENU_FRAME, 1 for BODY_FRAME"<<endl;
                cin >> Move_frame; 
                cout << "Please input reference state [x y z yaw]: "<< endl;
                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> state_desired[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> state_desired[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> state_desired[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cin >> state_desired[3];
            }
        }else if(Control_Mode == 999)
        {
            // 切换至offboard模式
            while(current_state.mode != "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "OFFBOARD";
                set_mode_client.call(mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;
                //执行回调函数
                ros::spinOnce();
                ros::Duration(1.0).sleep();
            }
            // 解锁
            while(!current_state.armed)
            {
                arm_cmd.request.value = true;
                arming_client.call(arm_cmd);
                cout << "Arming..." <<endl;
                //执行回调函数
                ros::spinOnce();
                ros::Duration(1.0).sleep();
            }
            cout << "Set to OFFBOARD Mode Susscess!!!" <<endl;
            cout << "Arm Susscess!!!" <<endl;
        }

        switch (Control_Mode)
        {
            case prometheus_msgs::ControlCommand::Idle:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Idle;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::Takeoff:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::Hold:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;
    
            case prometheus_msgs::ControlCommand::Land:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::Move:
                if(Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
                {
                    float time_trajectory = 0.0;

                    while(time_trajectory < trajectory_total_time)
                    {
                        Command_Now.header.stamp = ros::Time::now();
                        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                        Command_Now.Command_ID = Command_Now.Command_ID + 1;

                        if(Trjectory_mode == 0)
                        {
                            Command_Now.Reference_State = Controller_Test.Circle_trajectory_generation(time_trajectory);
                        }else if(Trjectory_mode == 1)
                        {
                            Command_Now.Reference_State = Controller_Test.Eight_trajectory_generation(time_trajectory);
                        }else if(Trjectory_mode == 2)
                        {
                            Command_Now.Reference_State = Controller_Test.Step_trajectory_generation(time_trajectory);
                        }

                        move_pub.publish(Command_Now);
                        time_trajectory = time_trajectory + 0.01;

                        cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

                        ros::Duration(0.01).sleep();
                    }

                }else
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID = Command_Now.Command_ID + 1;
                    Command_Now.Reference_State.Move_mode  = Move_mode;
                    Command_Now.Reference_State.Move_frame = Move_frame;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_Now);
                }
                break;
            
            case prometheus_msgs::ControlCommand::Disarm:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::User_Mode1:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::User_Mode1;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;
            
            case prometheus_msgs::ControlCommand::User_Mode2:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::User_Mode2;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;
        }
        
        ROS_INFO("..................................");
        
        sleep(2.0);
    }

    return 0;
}

void generate_com(int Move_mode, float state_desired[4])
{
    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if((Move_mode & 0b10) == 0) //xy channel
    {
        Command_Now.Reference_State.position_ref[0] = state_desired[0];
        Command_Now.Reference_State.position_ref[1] = state_desired[1];
        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;
        Command_Now.Reference_State.velocity_ref[0] = state_desired[0];
        Command_Now.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((Move_mode & 0b01) == 0) //z channel
    {
        Command_Now.Reference_State.position_ref[2] = state_desired[2];
        Command_Now.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_Now.Reference_State.position_ref[2] = 0;
        Command_Now.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;


    Command_Now.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
}
