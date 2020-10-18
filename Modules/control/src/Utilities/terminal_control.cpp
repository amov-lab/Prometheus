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
#include <nav_msgs/Path.h>

#define TRA_WINDOW 2000
#define NODE_NAME "terminal_control"

using namespace std;

prometheus_msgs::ControlCommand Command_Now;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
ros::Publisher move_pub;
ros::Publisher ref_trajectory_pub;
void generate_com(int Move_mode, float state_desired[4]);
void Draw_in_rviz(const prometheus_msgs::PositionReference& pos_ref, bool draw_trajectory);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh;

    move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //【发布】参考轨迹
    ref_trajectory_pub = nh.advertise<nav_msgs::Path>("/prometheus/reference_trajectory", 10);

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
    Command_Now.source = NODE_NAME;
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
            cout << "Input the Move_mode: 0 for xy/z position control, 1 for xy_pos_z_vel control, 2 for xy_vel_z_pos control, 3 for xy/z velocity control, 4 for trajectory tracking"<<endl;
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
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.yaw_ref = 999;
            move_pub.publish(Command_Now);
        }

        switch (Control_Mode)
        {
            case prometheus_msgs::ControlCommand::Idle:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Idle;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::Takeoff:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::Hold:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                move_pub.publish(Command_Now);
                break;
    
            case prometheus_msgs::ControlCommand::Land:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
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
                        Command_Now.source = NODE_NAME;

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

                        Draw_in_rviz(Command_Now.Reference_State, true);

                        ros::Duration(0.01).sleep();
                    }
                    
                }else
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Reference_State.Move_mode  = Move_mode;
                    Command_Now.Reference_State.Move_frame = Move_frame;
                    Command_Now.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
        
                    move_pub.publish(Command_Now);
                }
                break;
            
            case prometheus_msgs::ControlCommand::Disarm:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                move_pub.publish(Command_Now);
                break;

            case prometheus_msgs::ControlCommand::User_Mode1:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::User_Mode1;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                move_pub.publish(Command_Now);
                break;
            
            case prometheus_msgs::ControlCommand::User_Mode2:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = prometheus_msgs::ControlCommand::User_Mode2;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                move_pub.publish(Command_Now);
                break;
        }
        
        ROS_INFO("..................................");
        
        sleep(1.0);
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

void Draw_in_rviz(const prometheus_msgs::PositionReference& pos_ref, bool draw_trajectory)
{
    geometry_msgs::PoseStamped reference_pose;

    reference_pose.header.stamp = ros::Time::now();
    reference_pose.header.frame_id = "world";

    reference_pose.pose.position.x = pos_ref.position_ref[0];
    reference_pose.pose.position.y = pos_ref.position_ref[1];
    reference_pose.pose.position.z = pos_ref.position_ref[2];

    if(draw_trajectory)
    {
        posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
        if(posehistory_vector_.size() > TRA_WINDOW){
            posehistory_vector_.pop_back();
        }
        
        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }else
    {
        posehistory_vector_.clear();
        
        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }
}