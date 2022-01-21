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
#include <nav_msgs/Path.h>
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


Vec3 pos0 = Vec3(0, 0, 2); //position
Vec3 vel0 = Vec3(0, 0, 0); //velocity
Vec3 acc0 = Vec3(0, 0, 0); //acceleration

//define the goal state:
Vec3 posf = Vec3(1, 0, 1); //position
Vec3 velf = Vec3(0, 0, 1); //velocity
Vec3 accf = Vec3(0, 0, 0); //acceleration

int flag_get_goal = 0;
float time_trajectory;
//define the duration: TF不能打死
double Tf = 5.0;
float Alpha[3],Beta[3],Gamma[3];

nav_msgs::Path Optimal_Path;

ros::Publisher ref_tra_pub;
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


void pub_ref_trajectory();
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);

    //　能用，但距离使用还差前端　和　后端约束

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10, goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    ref_tra_pub = nh.advertise<nav_msgs::Path>("/prometheus/motion_planning/ref_trajectory", 10);


    int start_flag;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Motion Planning Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "Please enter 1 for takeoff"<<endl;
    cin >> start_flag;

    // 起飞
    if( start_flag == 1)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = 1;
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

            //　根据目标点重新预估时间,希望平均速度为1m/s
            Tf = sqrt( pow(posf[0] - pos0[0], 2) + pow(posf[1] - pos0[1], 2) + pow(posf[2] - pos0[2], 2)) / 1.0;

            traj.Generate(Tf);

            for(int i = 0; i < 3; i++)
            {
                Alpha[i] = traj.GetAxisParamAlpha(i);
                Beta[i] = traj.GetAxisParamBeta(i);
                Gamma[i] = traj.GetAxisParamGamma(i);

                cout << "Axis #" << i << "\n";
                cout << "\tAlpha = " << Alpha[i];
                cout << "\tBeta = "  << Beta[i];
                cout << "\tGamma = " << Gamma[i];
                cout << "\n";
            }

            cout << "Tf =  " << Tf << " [s] " << endl;
            cout << "Total cost = " << traj.GetCost() << "\n";
            cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;
            cout << "Input feasible? " << GetInputFeasibilityResultName(traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec)) << "\n";
            cout << "Position feasible? " << GetStateFeasibilityResultName(traj.CheckPositionFeasibility(floorPos, floorNormal)) << "\n";

            pub_ref_trajectory();

            time_trajectory = 0.0;
            while(time_trajectory < Tf)
            {
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode         = prometheus_msgs::ControlCommand::Move;
                Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;
                Command_Now.Reference_State.time_from_start = time_trajectory;
                for(int i = 0; i < 3; i++)
                {
                    //时间戳怎么解决？
                    Command_Now.Reference_State.position_ref[i]     = 1/120.0 * Alpha[i] * pow(time_trajectory,5) + 1/24.0 * Beta[i] * pow(time_trajectory,4) + 1/6.0 * Gamma[i] * pow(time_trajectory,3) + 1/2.0 * acc0[i] * pow(time_trajectory,2) + vel0[i] * time_trajectory + pos0[i];
                    Command_Now.Reference_State.velocity_ref[i]     =  1/24.0 * Alpha[i] * pow(time_trajectory,4) +  1/6.0 * Beta[i] * pow(time_trajectory,3) + 1/2.0 * Gamma[i] * pow(time_trajectory,2) + acc0[i] * time_trajectory + vel0[i];
                    Command_Now.Reference_State.acceleration_ref[i] =   1/6.0 * Alpha[i] * pow(time_trajectory,3) +  1/2.0 * Beta[i] * pow(time_trajectory,2) + Gamma[i] * time_trajectory + acc0[i];
                }
                

                Command_Now.Reference_State.yaw_ref = 0.0;

                command_pub.publish(Command_Now);
                time_trajectory = time_trajectory + 0.01;
                ros::Duration(0.01).sleep();


                // cout << "Motion Planning Result:"<<endl;
                // cout << "Trajectory tracking: "<< time_trajectory << " / " << Tf  << " [ s ]" <<endl;
                // cout << "desired_point: "   << Command_Now.Reference_State.position_ref[0] << " [m] "
                //                             << Command_Now.Reference_State.position_ref[1] << " [m] "
                //                             << Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;  
                // cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;

            }


        }

    }

    return 0;
}

void pub_ref_trajectory()
{
    Optimal_Path.header.frame_id = "world";
    Optimal_Path.header.stamp = ros::Time::now();
    Optimal_Path.poses.clear();


    time_trajectory = 0.0;
    int k = 0;
    while(time_trajectory < Tf)
    {
        geometry_msgs::PoseStamped way_point;
        way_point.header.frame_id = "world";
        way_point.pose.position.x = 1/120.0 * Alpha[0] * pow(time_trajectory,5) + 1/24.0 * Beta[0] * pow(time_trajectory,4) + 1/6.0 * Gamma[0] * pow(time_trajectory,3) + 1/2.0 * acc0[0] * pow(time_trajectory,2) + vel0[0] * time_trajectory + pos0[0];
        way_point.pose.position.y = 1/120.0 * Alpha[1] * pow(time_trajectory,5) + 1/24.0 * Beta[1] * pow(time_trajectory,4) + 1/6.0 * Gamma[1] * pow(time_trajectory,3) + 1/2.0 * acc0[1] * pow(time_trajectory,2) + vel0[1] * time_trajectory + pos0[1];
        way_point.pose.position.z = 1/120.0 * Alpha[2] * pow(time_trajectory,5) + 1/24.0 * Beta[2] * pow(time_trajectory,4) + 1/6.0 * Gamma[2] * pow(time_trajectory,3) + 1/2.0 * acc0[2] * pow(time_trajectory,2) + vel0[2] * time_trajectory + pos0[2];
        Optimal_Path.poses.push_back(way_point);
        time_trajectory = time_trajectory + 0.01;
    }

    ref_tra_pub.publish(Optimal_Path);

}
