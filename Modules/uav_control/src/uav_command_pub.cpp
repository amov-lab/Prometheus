//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <prometheus_msgs/UAVSetup.h>
#include <prometheus_msgs/UAVControlState.h>

#include "controller_test.h"

using namespace std;
// 获取无人机当前信息
prometheus_msgs::UAVState uav_state;

prometheus_msgs::UAVControlState uav_control_state;
// 发布的指令
prometheus_msgs::UAVCommand agent_command;

// 发布指点飞行
ros::Publisher uav_command_pub;

void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr& msg)
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

    //【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav"+std::to_string(uav_id)+"/prometheus/state", 1, uav_state_cb);
    
    //【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(uav_id) + "/prometheus/control_state", 1, uav_control_state_cb);

    //【发布】mavros接口调用指令(-> uav_control.cpp)
    ros::Publisher uav_setup_pub = nh.advertise<prometheus_msgs::UAVSetup>("/uav" + std::to_string(uav_id) + "/prometheus/setup", 1);

    //【发布】UAVCommand
    ros::Publisher uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav"+std::to_string(uav_id)+ "/prometheus/command", 1);

    //用于控制器测试的类，功能例如：生成圆形轨迹，８字轨迹等
    Controller_Test Controller_Test;    // 打印参数
    Controller_Test.printf_param();

    int CMD = 0;
    float state_desired[4];

    // prometheus_msgs::UAVCommand agent_command;
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
    // 发布指令初始值
    uav_command_pub.publish(agent_command);
    float time_trajectory = 0.0;
    int start_flag = 0;
    
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "Please enter 1 to disarm and takeoff the drone ..."<<endl;
    cin >> CMD;

    // 跳过遥控器逻辑，直接进入COMMAND模式
    prometheus_msgs::UAVSetup uav_setup;
    while(ros::ok())
    {
        ros::spinOnce();

        if(!uav_state.armed)
        {
            uav_setup.cmd = prometheus_msgs::UAVSetup::ARMING;
            uav_setup.arming = true;
            uav_setup_pub.publish(uav_setup);
            cout << "Arming ..."<<endl;
            sleep(1.0);
        }

        ros::spinOnce();

        if(uav_state.armed)
        {
            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            agent_command.position_ref[0] = 0.0;
            agent_command.position_ref[1] = 0.0;
            agent_command.position_ref[2] = 1.0;
            agent_command.yaw_ref = 0.0;
            agent_command.Command_ID = agent_command.Command_ID + 1;
            uav_command_pub.publish(agent_command);
            cout << "TAKEOFF ..."<<endl;

            if(uav_state.mode != "OFFBOARD")
            {
                uav_setup.cmd = prometheus_msgs::UAVSetup::SET_PX4_MODE;
                uav_setup.px4_mode = "OFFBOARD";
                uav_setup_pub.publish(uav_setup);
                cout << "Enable OFFBOARD mode ..."<<endl;
                sleep(1.0);
            }
            sleep(1.0);
        }
        
        if(uav_state.position[2] > 0.8)
        {
            break;
        }
    }

    while(ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the CMD: 1 for Move(XYZ_POS),2 for Move(XYZ_POS_BODY), 3 for Current_Pos_Hover, 4 for Land， 5 for Trajectory, 6 for Move(XYZ_VEL_YAW_RATE_BODY)..."<<endl;
        cin >> CMD;

        switch (CMD)
        {
        case 1:
                        
            cout << "Move in ENU frame, Pls input the desired position and yaw angle"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]"<<endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:"<<endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3]/180.0*M_PI;

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            agent_command.position_ref[0] = state_desired[0];
            agent_command.position_ref[1] = state_desired[1];
            agent_command.position_ref[2] = state_desired[2];
            agent_command.yaw_ref = state_desired[3];
            agent_command.Command_ID = agent_command.Command_ID + 1;
            uav_command_pub.publish(agent_command);

            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "yaw_des : " << state_desired[3]/M_PI*180.0 <<" [ deg ] " << start_flag << endl;
            break;

        case 2:
            
            cout << "Move in BODY frame, Pls input the desired position and yaw angle"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]"<<endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:"<<endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3]/180.0*M_PI;

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

            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "yaw_des : " << state_desired[3]/M_PI*180.0 <<" [ deg ] "<< endl;
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
            cout << "For safety, please move the drone near to the trajectory start point firstly!!!"<<endl;
            cout << "Please choose the trajectory type: 0 for Circle, 1 for Eight Shape, 2 for Step, 3 for Line"<<endl;
            cin >> Trjectory_mode;  
            cout << "Input the trajectory_total_time:"<<endl;
            cin >> trajectory_total_time;

            time_trajectory = 0.0;

            while(time_trajectory < trajectory_total_time)
            {

                if(Trjectory_mode == 0)
                {
                    agent_command = Controller_Test.Circle_trajectory_generation(time_trajectory);
                }else if(Trjectory_mode == 1)
                {
                    agent_command = Controller_Test.Eight_trajectory_generation(time_trajectory);
                }else if(Trjectory_mode == 2)
                {
                    agent_command = Controller_Test.Step_trajectory_generation(time_trajectory);
                }else if(Trjectory_mode == 3)
                {
                    agent_command = Controller_Test.Line_trajectory_generation(time_trajectory);
                }
                
                agent_command.header.stamp = ros::Time::now();
                agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                agent_command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
                uav_command_pub.publish(agent_command);

                time_trajectory = time_trajectory + 0.01;
                cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

                ros::Duration(0.01).sleep();
            } 
            break;
        case 6:
            
            cout << "Move in BODY frame, Pls input the desired vel and yaw rate"<<endl;
            cout << "desired state: --- x [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m/s]"<<endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw_rate [deg/s]:"<<endl;
            cin >> state_desired[3];
            // state_desired[3] = state_desired[3]/180.0*M_PI;

            agent_command.header.stamp = ros::Time::now();
            agent_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            agent_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            agent_command.velocity_ref[0] = state_desired[0];
            agent_command.velocity_ref[1] = state_desired[1];
            agent_command.velocity_ref[2] = state_desired[2];
            agent_command.Yaw_Rate_Mode = true;
            agent_command.yaw_rate_ref = state_desired[3];
            agent_command.Command_ID = agent_command.Command_ID + 1;
            uav_command_pub.publish(agent_command);

            cout << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] "<< state_desired[1] <<" [ m/s ] "<< state_desired[2] <<" [ m ] "<< endl;
            cout << "yaw_rate_des : " << state_desired[3] <<" [ deg/s ] "<< endl;
            break;
        
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
