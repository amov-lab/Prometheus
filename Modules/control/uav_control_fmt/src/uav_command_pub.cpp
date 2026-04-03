// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <prometheus_msgs/UAVSetup.h>
#include <prometheus_msgs/UAVControlState.h>
#include <nav_msgs/Path.h>

#include "controller_test.h"
#include "printf_utils.h"

using namespace std;
#define TRA_WINDOW 2000
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;
prometheus_msgs::UAVCommand agent_command;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

// 如果要使用地面站PrometheusGround控制，需要将此值改为true，否则改为false
bool is_ground_station_control = false;
bool flag = false;

void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_command_pub");
    ros::NodeHandle nh("~");

    int uav_id;
    bool sim_mode;
    nh.param("uav_id", uav_id, 1);
    nh.param("sim_mode", sim_mode, true);
    nh.getParam("/communication_bridge/trajectory_ground_control",is_ground_station_control);

    string uav_name = "";

    //【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>(uav_name + "/prometheus/state", 1, uav_state_cb);

    //【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>(uav_name + "/prometheus/control_state", 1, uav_control_state_cb);

    //【发布】UAVCommand
    ros::Publisher ref_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/reference_trajectory", 10);

    //【发布】UAVCommand
    ros::Publisher uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>(uav_name + "/prometheus/command", 1);

    //用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
    Controller_Test Controller_Test;
    Controller_Test.printf_param();

    int CMD = 0;
    float state_desired[4];

    agent_command.header.stamp = ros::Time::now();
    agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    agent_command.position_ref[0] = 0.0;
    agent_command.position_ref[1] = 0.0;
    agent_command.position_ref[2] = 0.0;
    agent_command.velocity_ref[0] = 0.0;
    agent_command.velocity_ref[1] = 0.0;
    agent_command.velocity_ref[2] = 0.0;
    agent_command.acceleration_ref[0] = 0.0;
    agent_command.acceleration_ref[1] = 0.0;
    agent_command.acceleration_ref[2] = 0.0;
    agent_command.att_ref[0] = 0.0;
    agent_command.att_ref[1] = 0.0;
    agent_command.att_ref[2] = 0.0;
    agent_command.att_ref[3] = 0.0;
    agent_command.Yaw_Rate_Mode = false;
    agent_command.yaw_ref = 0.0;
    agent_command.yaw_rate_ref = 0.0;
    agent_command.Command_ID = 0;

    float time_trajectory = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();

        if (uav_control_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            cout << YELLOW << "Please switch to COMMAND_CONTROL mode first" << TAIL << endl;
        }

        if (!is_ground_station_control)
        {
            cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
            cout << GREEN << "Please choose the CMD: 1 for Move(XYZ_POS),2 for Move(XYZ_POS_BODY), 3 for Current_Pos_Hover, 4 for Land, 5 for Trajectory, 6 for Move(XYZ_VEL_YAW_RATE_BODY)..." << TAIL << endl;
            cin >> CMD;
        }
        else
        {
            CMD = 5;
        }

        switch (CMD)
        {
        case 1:

            cout << "Move in ENU frame, Pls input the desired position and yaw angle" << endl;
            cout << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            agent_command.position_ref[0] = state_desired[0];
            agent_command.position_ref[1] = state_desired[1];
            agent_command.position_ref[2] = state_desired[2];
            agent_command.yaw_ref = state_desired[3];
            agent_command.Command_ID = agent_command.Command_ID + 1;
            uav_command_pub.publish(agent_command);

            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;

        case 2:

            cout << "Move in BODY frame, Pls input the desired position and yaw angle" << endl;
            cout << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
            agent_command.position_ref[0] = state_desired[0];
            agent_command.position_ref[1] = state_desired[1];
            agent_command.position_ref[2] = state_desired[2];
            agent_command.Yaw_Rate_Mode = true;
            agent_command.yaw_rate_ref = state_desired[3];
            agent_command.Command_ID = agent_command.Command_ID + 1;
            uav_command_pub.publish(agent_command);

            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;

        case 3:

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_command_pub.publish(agent_command);
            break;

        case 4:

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            uav_command_pub.publish(agent_command);

            break;

        case 5:

            static int Trjectory_mode;
            static int trajectory_total_time;
            if (!is_ground_station_control)
            {
                cout << "For safety, please move the drone near to the trajectory start point firstly!!!" << endl;
                cout << "Please choose the trajectory type: 0 for Circle, 1 for Eight Shape, 2 for Step, 3 for Line" << endl;
                cin >> Trjectory_mode;
                cout << "Input the trajectory_total_time:" << endl;
                cin >> trajectory_total_time;
            }
            else
            {
                while (1)
                {
                    nh.getParam("/communication_bridge/trajectory_flag", flag);
                    if (flag){
                        break;
                    }
                    usleep(10);
                }
                nh.getParam("/communication_bridge/trajectory_mode", Trjectory_mode);
                nh.getParam("/communication_bridge/trajectory_time", trajectory_total_time);
                nh.setParam("/communication_bridge/trajectory_flag",false);
            }

            time_trajectory = 0.0;

            while (time_trajectory < trajectory_total_time)
            {

                if (Trjectory_mode == 0)
                {
                    agent_command = Controller_Test.Circle_trajectory_generation(time_trajectory);
                }
                else if (Trjectory_mode == 1)
                {
                    agent_command = Controller_Test.Eight_trajectory_generation(time_trajectory);
                }
                else if (Trjectory_mode == 2)
                {
                    agent_command = Controller_Test.Step_trajectory_generation(time_trajectory);
                }
                else if (Trjectory_mode == 3)
                {
                    agent_command = Controller_Test.Line_trajectory_generation(time_trajectory);
                }

                agent_command.header.stamp = ros::Time::now();
                agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                agent_command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
                uav_command_pub.publish(agent_command);

                time_trajectory = time_trajectory + 0.01;
                cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

                geometry_msgs::PoseStamped reference_pose;

                reference_pose.header.stamp = ros::Time::now();
                reference_pose.header.frame_id = "world";

                reference_pose.pose.position.x = agent_command.position_ref[0];
                reference_pose.pose.position.y = agent_command.position_ref[1];
                reference_pose.pose.position.z = agent_command.position_ref[2];

                posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
                if (posehistory_vector_.size() > TRA_WINDOW)
                {
                    posehistory_vector_.pop_back();
                }

                nav_msgs::Path reference_trajectory;
                reference_trajectory.header.stamp = ros::Time::now();
                reference_trajectory.header.frame_id = "world";
                reference_trajectory.poses = posehistory_vector_;
                ref_trajectory_pub.publish(reference_trajectory);

                ros::Duration(0.01).sleep();
            }
            break;
        case 6:

            cout << "Move in BODY frame, Pls input the desired vel and yaw rate" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw_rate [deg/s]:" << endl;
            cin >> state_desired[3];

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            agent_command.velocity_ref[0] = state_desired[0];
            agent_command.velocity_ref[1] = state_desired[1];
            agent_command.velocity_ref[2] = state_desired[2];
            agent_command.Yaw_Rate_Mode = true;
            agent_command.yaw_rate_ref = state_desired[3] / 180.0 * M_PI;
            agent_command.Command_ID = agent_command.Command_ID + 1;
            uav_command_pub.publish(agent_command);

            cout << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_rate_des : " << state_desired[3] << " [ deg/s ] " << endl;
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
