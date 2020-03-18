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
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
nav_msgs::Path path;
int Num_total_wp;
int wp_id;

prometheus_msgs::PositionReference pos_cmd;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令

ros::Publisher command_pub;
int flag_get_cmd = 0;
geometry_msgs::PoseStamped goal;
prometheus_msgs::DroneState _DroneState;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int posehistory_window_ = 1000;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    flag_get_cmd = 1;
    path = *msg;
    Num_total_wp = path.poses.size();
    wp_id = 0;
    cout << "Get a new path!"<<endl;
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning");
    ros::NodeHandle nh("~");
    ros::Subscriber path_sub =nh.subscribe<nav_msgs::Path>("/planning/path_cmd", 50, path_cb);
    
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/planning/goal", 10,goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 频率 [10Hz]
    ros::Rate rate(10.0);

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
    command_pub.publish(Command_Now);

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

    
    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        if( flag_get_cmd == 0)
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Hold;
            Command_Now.Command_ID                          = 1;

            command_pub.publish(Command_Now);
            cout << "waiting for trajectory" << endl;
        }else
        {
            //只要当前航点中有未执行完的航点，就继续执行
            while( wp_id < Num_total_wp )
            {
                float desired_yaw;  //[rad]
                
                desired_yaw = 0.0;

                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
                Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
                Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
                Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.position_ref[0]     = path.poses[wp_id].pose.position.x ;
                Command_Now.Reference_State.position_ref[1]     = path.poses[wp_id].pose.position.y;
                Command_Now.Reference_State.position_ref[2]     = path.poses[wp_id].pose.position.z;
                Command_Now.Reference_State.yaw_ref             = desired_yaw;
                wp_id++;

                command_pub.publish(Command_Now);
                
                cout << "desired_yaw: " << desired_yaw / M_PI * 180 << " [deg] "<<endl;
                cout << "desired_point: " << path.poses[wp_id].pose.position.x << " [m] "<< path.poses[wp_id].pose.position.y << " [m] "<< path.poses[wp_id].pose.position.z << " [m] "<<endl;     
            
                //计算当前位置与下一个航点的距离，预估此处等待时间,最大速度为0.2m/s
                double distance_to_next_wp = sqrt(  pow(_DroneState.position[0] - path.poses[wp_id].pose.position.x, 2) +
                                                    pow(_DroneState.position[1] - path.poses[wp_id].pose.position.y, 2) +
                                                    pow(_DroneState.position[2] - path.poses[wp_id].pose.position.z, 2)  );
                float wait_time = distance_to_next_wp / 0.2;

                int total_k = wait_time/0.01;

                int k = 0;

                while(distance_to_next_wp > 0.1 && k<total_k)
                {
                    distance_to_next_wp = sqrt( pow(_DroneState.position[0] - path.poses[wp_id].pose.position.x, 2) +
                                                pow(_DroneState.position[1] - path.poses[wp_id].pose.position.y, 2) +
                                                pow(_DroneState.position[2] - path.poses[wp_id].pose.position.z, 2)  );   
                    sleep(0.01);
                    k = k+1;

                    cout << "Moving to Waypoint: [ " << wp_id << " / "<< Num_total_wp<< " ] "<<endl;
                }

                
            }
        }
        
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Planning<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;

        rate.sleep();
    }

    return 0;

}
