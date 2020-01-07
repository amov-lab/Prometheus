/***************************************************************************************************************************
* move.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  test function for sending command.msg
***************************************************************************************************************************/
#include <ros/ros.h>

#include <iostream>
#include <prometheus_msgs/ControlCommand.h>
#include <command_to_mavros.h>

using namespace std;

prometheus_msgs::ControlCommand Command_Now;

void generate_com(int Move_mode, float state_desired[4]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    int Control_Mode;
    int Move_mode;
    int Move_frame;
    int Trjectory_mode;
    float state_desired[4];

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
        cout << "Input the Mode: 0 for Idle, 1 for Takeoff, 2 for Hold, 3 for Land, 4 for Move, 5 for User_Mode1, 6 for User_Mode2, 7 for User_Mode3"<<endl;
        cin >> Control_Mode;

        if(Control_Mode == prometheus_msgs::ControlCommand::Move)
        {
            cout << "Input the Move_mode: 0 for xy/z position control, 3 for xy/z velocity control, 4 for trajectory tracking"<<endl;
            cin >> Move_mode;

            if(Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
            {
                cout << "Input the Trajectory: 0 for Circle, 1 for Eight Shape, 2 for Step"<<endl;
                cin >> Trjectory_mode;  
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
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;

                if(Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
                {
                    if(Trjectory_mode == 0)
                    {
                        //Circle();
                    }else if(Trjectory_mode == 1)
                    {
                        //Eight();
                    }else if(Trjectory_mode == 2)
                    {
                        //Step();
                    }
                }else
                {
                    generate_com(Move_mode, state_desired);
                }

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
            
            case prometheus_msgs::ControlCommand::User_Mode3:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::User_Mode3;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                move_pub.publish(Command_Now);
                break;
        }

        sleep(0.5);
    }

    return 0;
}

void generate_com(int Move_mode, float state_desired[4])
{

    Command_Now.Reference_State.Move_mode  = Move_mode;

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
