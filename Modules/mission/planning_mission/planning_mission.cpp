//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/PositionReference.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
using namespace std;

#define MIN_DIS 0.1
#define FLY_HEIGHT 1.0
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;

geometry_msgs::PoseStamped goal;                              // goal                    
// 停止指令
std_msgs::Int8 stop_cmd; 

struct local_planner
{
    // 期望速度
    geometry_msgs::Point desired_vel;  
}APF;

struct global_planner
{
    // 规划路径
    nav_msgs::Path path_cmd;
    int Num_total_wp;
    int wp_id;  
}A_star;

struct fast_planner
{
    prometheus_msgs::PositionReference fast_planner_cmd;          // fast planner cmd

}fast_planner;


int flag_get_cmd = 0;
int flag_get_goal = 0;
float desired_yaw = 0;  //[rad]
float distance_to_goal = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void APF_planner();
void A_star_planner();
void Fast_planner();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void global_planner_cmd_cb(const nav_msgs::Path::ConstPtr& msg)
{
    flag_get_cmd = 1;
    A_star.path_cmd = *msg;
    A_star.Num_total_wp = A_star.path_cmd.poses.size();
    A_star.wp_id = 0;
}
void stop_cmd_cb(const std_msgs::Int8::ConstPtr& msg)
{
    stop_cmd = *msg;
}
void local_planner_cmd_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    flag_get_cmd = 1;
    APF.desired_vel = *msg;
}
void fast_planner_cmd_cb(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    flag_get_cmd = 1;
    fast_planner.fast_planner_cmd = *msg;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    distance_to_goal = sqrt(  pow(_DroneState.position[0] - goal.pose.position.x, 2) 
                            + pow(_DroneState.position[1] - goal.pose.position.y, 2) );
}
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    flag_get_goal = 1;
    cout << "Get a new goal!"<<endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_mission");
    ros::NodeHandle nh("~");
    
    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    
    //【订阅】来自planning的指令
    ros::Subscriber global_planner_sub =    nh.subscribe<nav_msgs::Path>("/prometheus/planning/path_cmd", 50, global_planner_cmd_cb);
    ros::Subscriber local_planner_sub  =    nh.subscribe<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 50, local_planner_cmd_cb);
    ros::Subscriber fast_planner_sub   =    nh.subscribe<prometheus_msgs::PositionReference>("/prometheus/planning/position_cmd", 50, fast_planner_cmd_cb);
    ros::Subscriber stop_cmd_sub = nh.subscribe<std_msgs::Int8>("/prometheus/planning/stop_cmd", 10, stop_cmd_cb);  
    stop_cmd.data = 0;

    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10,goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Local Planning Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the planning method: 1 for APF, 2 for A*, 3 for Fast planner"<<endl;
        cin >> start_flag;
    }

    // 起飞
    while( _DroneState.position[2] < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = 1;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>> Planning Mission <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        if( flag_get_cmd == 0)
        {
            cout << "Waiting for trajectory" << endl;
        }else if (distance_to_goal < MIN_DIS)
        {
            // 抵达目标附近，则停止速度控制，改为位置控制
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.position_ref[0]     = goal.pose.position.x;
            Command_Now.Reference_State.position_ref[1]     = goal.pose.position.y;
            if (start_flag == 1)
            {
                Command_Now.Reference_State.position_ref[2]     = FLY_HEIGHT;
            }else
            {
                Command_Now.Reference_State.position_ref[2]     = goal.pose.position.z;
            }
            Command_Now.Reference_State.yaw_ref             = desired_yaw;
            command_pub.publish(Command_Now);
            cout << "Arrived the goal, waiting for a new goal... " << endl;
            cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
            cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;
            flag_get_goal = 0;
            while (flag_get_goal == 0)
            {
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }
        }else if(stop_cmd.data == 1)
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Hold;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;

            command_pub.publish(Command_Now);
            cout << "Dangerous! Hold there." << endl; 

            ros::Duration(0.1).sleep();
        }else
        {
            // 运动阶段，执行规划指令
            if (start_flag == 1)
            {
                APF_planner();
                ros::Duration(0.05).sleep();
            }else if (start_flag == 2)
            {
                A_star_planner();
                ros::Duration(0.1).sleep();
            }else if (start_flag == 3)
            {
                Fast_planner();
                ros::Duration(0.05).sleep();
            }
            cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
            cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;
        }
    }

    return 0;

}

void APF_planner()
{
    // 根据速度大小决定是否更新期望偏航角， 更新采用平滑滤波的方式，系数可调
    if( sqrt(APF.desired_vel.x*APF.desired_vel.x + APF.desired_vel.y*APF.desired_vel.y)  >  0.1   )
    {
        float next_desired_yaw = atan2(APF.desired_vel.y, APF.desired_vel.x);
        desired_yaw = (0.6*desired_yaw + 0.4*next_desired_yaw);
    }
    
    // 高度改为定高飞行
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.velocity_ref[0]     = APF.desired_vel.x;
    Command_Now.Reference_State.velocity_ref[1]     = APF.desired_vel.y;
    Command_Now.Reference_State.position_ref[2]     = FLY_HEIGHT;
    Command_Now.Reference_State.yaw_ref             = desired_yaw;
    command_pub.publish(Command_Now);
    cout << "APF planner:"<<endl;
    cout << "desired_vel: " << APF.desired_vel.x << " [m/s] "<< APF.desired_vel.y << " [m/s] "<< APF.desired_vel.z << " [m/s] "<<endl;
    cout << "desired_yaw: " << desired_yaw / M_PI * 180 << " [deg] "<<endl;
}

void A_star_planner()
{
    //只要当前航点中有未执行完的航点，就继续执行
    while( A_star.wp_id < A_star.Num_total_wp )
    {
        // 更新的话加滤波平滑期望航向角
        float next_desired_yaw = atan2(A_star.path_cmd.poses[A_star.wp_id].pose.position.y - _DroneState.position[1], 
                                        A_star.path_cmd.poses[A_star.wp_id].pose.position.x - _DroneState.position[0]);
        desired_yaw = (0.6*desired_yaw + 0.4*next_desired_yaw);

        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = A_star.path_cmd.poses[A_star.wp_id].pose.position.x ;
        Command_Now.Reference_State.position_ref[1]     = A_star.path_cmd.poses[A_star.wp_id].pose.position.y;
        Command_Now.Reference_State.position_ref[2]     = A_star.path_cmd.poses[A_star.wp_id].pose.position.z;
        Command_Now.Reference_State.yaw_ref             = desired_yaw;
        A_star.wp_id++;

        command_pub.publish(Command_Now);
        cout << "A star planner:"<<endl;
        cout << "Moving to Waypoint: [ " << A_star.wp_id << " / "<< A_star.Num_total_wp<< " ] "<<endl;
        cout << "desired_point: "   << A_star.path_cmd.poses[A_star.wp_id].pose.position.x << " [m] "
                                    << A_star.path_cmd.poses[A_star.wp_id].pose.position.y << " [m] "
                                    << A_star.path_cmd.poses[A_star.wp_id].pose.position.z << " [m] "<<endl;     
        
        //计算当前位置与下一个航点的距离，预估此处等待时间,最大速度为0.1m/s
        double distance_to_next_wp = sqrt(  pow(_DroneState.position[0] - A_star.path_cmd.poses[A_star.wp_id].pose.position.x, 2) +
                                            pow(_DroneState.position[1] - A_star.path_cmd.poses[A_star.wp_id].pose.position.y, 2) +
                                            pow(_DroneState.position[2] - A_star.path_cmd.poses[A_star.wp_id].pose.position.z, 2)  );
        float wait_time = distance_to_next_wp / 0.05;

        int total_k = wait_time/0.01;

        int k = 0;

        while(distance_to_next_wp > 0.05 && k<total_k)
        {
            double distance_to_next_wp = sqrt(  pow(_DroneState.position[0] - A_star.path_cmd.poses[A_star.wp_id].pose.position.x, 2) +
                                                pow(_DroneState.position[1] - A_star.path_cmd.poses[A_star.wp_id].pose.position.y, 2) +
                                                pow(_DroneState.position[2] - A_star.path_cmd.poses[A_star.wp_id].pose.position.z, 2)  );
            sleep(0.01);
            k = k+1;  
        }
    }
}

void Fast_planner()
{
    float next_desired_yaw      = atan2( fast_planner.fast_planner_cmd.velocity_ref[1] , 
                                         fast_planner.fast_planner_cmd.velocity_ref[0]);

    desired_yaw = (0.8*desired_yaw + 0.2*next_desired_yaw);
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.Reference_State = fast_planner.fast_planner_cmd;
    Command_Now.Reference_State.yaw_ref = desired_yaw;

    command_pub.publish(Command_Now);
    cout << "Fast planner:"<<endl;
    cout << "desired_point: "   << Command_Now.Reference_State.position_ref[0] << " [m] "
                                << Command_Now.Reference_State.position_ref[1] << " [m] "
                                << Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;  
    cout << "desired_yaw: " << desired_yaw / M_PI * 180 << " [deg] "<<endl;
}