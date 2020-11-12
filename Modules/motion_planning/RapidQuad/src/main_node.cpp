//头文件
#include <ros/ros.h>
#include <iostream>


#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/PositionReference.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>

// #include "message_utils.h"
#include "RapidTrajectoryGenerator.h"

using namespace std;
using namespace RapidQuadrocopterTrajectoryGenerator;

#define NODE_NAME "motion_planning"


prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
geometry_msgs::PoseStamped goal;
int flag_get_goal = 0;

float alpha[3],beta[3],gamma[3];

//Two simple helper function to make testing easier
const char* GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr)
{
    switch(fr)
    {
    case RapidTrajectoryGenerator::InputFeasible:             return "Feasible";
    case RapidTrajectoryGenerator::InputIndeterminable:       return "Indeterminable";
    case RapidTrajectoryGenerator::InputInfeasibleThrustHigh: return "InfeasibleThrustHigh";
    case RapidTrajectoryGenerator::InputInfeasibleThrustLow:  return "InfeasibleThrustLow";
    case RapidTrajectoryGenerator::InputInfeasibleRates:      return "InfeasibleRates";
    }
    return "Unknown!";
};

const char* GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr)
{
    switch(fr)
    {
    case RapidTrajectoryGenerator::StateFeasible:   return "Feasible";
    case RapidTrajectoryGenerator::StateInfeasible: return "Infeasible";
    }
    return "Unknown!";
};


void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    flag_get_goal = 1;
    cout << "Get a new goal!"<<endl;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10, goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 起飞
    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;
    while( _DroneState.position[2] < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    Vec3 pos0 = Vec3(0, 0, 2); //position
    Vec3 vel0 = Vec3(0, 0, 0); //velocity
    Vec3 acc0 = Vec3(0, 0, 0); //acceleration

    //define the goal state:
    Vec3 posf = Vec3(1, 0, 1); //position
    Vec3 velf = Vec3(0, 0, 1); //velocity
    Vec3 accf = Vec3(0, 0, 0); //acceleration

    //define the duration:
    double Tf = 1.3;

    double fmin = 5;//[m/s**2]
    double fmax = 25;//[m/s**2]
    double wmax = 20;//[rad/s]
    double minTimeSec = 0.02;//[s]

    //Define how gravity lies in our coordinate system
    Vec3 gravity = Vec3(0,0,-9.81);//[m/s**2]

    //Define the state constraints. We'll only check that we don't fly into the floor:
    Vec3 floorPos = Vec3(0,0,0);//any point on the boundary
    Vec3 floorNormal = Vec3(0,0,1);//we want to be in this direction of the boundary

    while (ros::ok())
    {
        flag_get_goal = 0;
        ros::spinOnce();

        if(flag_get_goal == 1)
        {
            //Define the trajectory starting state:
            pos0 = Vec3(_DroneState.position[0], _DroneState.position[1], _DroneState.position[2]); 
            vel0 = Vec3(_DroneState.velocity[0], _DroneState.velocity[1], _DroneState.velocity[2]); 
            acc0 = Vec3(0, 0, 0); //acceleration

            posf = Vec3(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z); 
            velf = Vec3(0.0, 0.0, 0.0); 
            accf = Vec3(0, 0, 0); //acceleration

            RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
            traj.SetGoalPosition(posf);
            traj.SetGoalVelocity(velf);
            traj.SetGoalAcceleration(accf);

            // Note: if you'd like to leave some states free, you can encode it like below.
            // Here we would be leaving the velocity in `x` (axis 0) free:
            //
            // traj.SetGoalVelocityInAxis(1,velf[1]);
            // traj.SetGoalVelocityInAxis(2,velf[2]);
            traj.Generate(Tf);

            for(int i = 0; i < 3; i++)
            {
                alpha[i] = traj.GetAxisParamAlpha(i);
                beta[i] = traj.GetAxisParamBeta(i);
                gamma[i] = traj.GetAxisParamGamma(i);

                cout << "Axis #" << i << "\n";
                cout << "\talpha = " << alpha[i];
                cout << "\tbeta = "  << beta[i];
                cout << "\tgamma = " << gamma[i];
                cout << "\n";
            }
            cout << "Total cost = " << traj.GetCost() << "\n";
            cout << "Input feasible? " << GetInputFeasibilityResultName(traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec)) << "\n";
            cout << "Position feasible? " << GetStateFeasibilityResultName(traj.CheckPositionFeasibility(floorPos, floorNormal)) << "\n";

            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode         = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State = fast_planner.fast_planner_cmd;

            for(int i = 0; i < 3; i++)
            {
                //时间戳怎么解决？
                Command_Now.Reference_State.position_ref[i] = alpha[i]/120

            }
            

            Command_Now.Reference_State.yaw_ref = 0.0;

            command_pub.publish(Command_Now);
            cout << "Motion Planning Result:"<<endl;
            cout << "desired_point: "   << Command_Now.Reference_State.position_ref[0] << " [m] "
                                        << Command_Now.Reference_State.position_ref[1] << " [m] "
                                        << Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;  
            cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
            cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;

        }

    }

    return 0;
}

