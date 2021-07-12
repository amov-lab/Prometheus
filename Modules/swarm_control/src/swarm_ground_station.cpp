//头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <math.h>

#include <prometheus_msgs/Message.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/LogMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "message_utils.h"
#include "formation_utils.h"


using namespace std;
#define NUM_POINT 2
#define MAX_UAV_NUM 50

//参数声明
int swarm_num_uav;
prometheus_msgs::DroneState State_uav[MAX_UAV_NUM+1];
prometheus_msgs::SwarmCommand Command_uav[MAX_UAV_NUM+1];
geometry_msgs::PoseStamped ref_pose_uav[MAX_UAV_NUM+1];

ros::Subscriber drone_state_sub[MAX_UAV_NUM+1];
ros::Subscriber command_sub[MAX_UAV_NUM+1];

void swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg, int id)
{
    Command_uav[id] = *msg;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int id)
{
    State_uav[id] = *msg;
}

void printf_swarm_state(int swarm_num, int uav_id, const prometheus_msgs::DroneState& _Drone_state, const prometheus_msgs::SwarmCommand& SwarmCommand);

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_ground_station");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num_uav", swarm_num_uav, 1);

    for(int i = 1; i <= swarm_num_uav; i++) 
    {
        // 订阅
        command_sub[i] = nh.subscribe<prometheus_msgs::SwarmCommand>("/uav" + std::to_string(i) + "/prometheus/swarm_command", 10, boost::bind(swarm_command_cb,_1,i));
        drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/uav" + std::to_string(i) + "/prometheus/drone_state", 10, boost::bind(drone_state_cb,_1,i));
    }

    while(ros::ok())
    {
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>> Swarm Ground Station <<<<<<<<<<<<<<<<<<< "<< endl;
        for(int i = 1; i <= swarm_num_uav; i++)
        {
            printf_swarm_state(swarm_num_uav, i, State_uav[i], Command_uav[i]);
        }
        sleep(2.0); // frequence
    }
    return 0;
}


void printf_swarm_state(int swarm_num, int uav_id, const prometheus_msgs::DroneState& _Drone_state, const prometheus_msgs::SwarmCommand& SwarmCommand)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    Eigen::MatrixXf formation;
    float x,y,z,yaw;
    cout <<">>>>>>>>>>>>>>>>>>>>>>UAV[" << uav_id <<  "] State" << "<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "UAV_id: ["<< uav_id <<"] ";

   //是否和飞控建立起连接
    if (_Drone_state.connected == true)
    {
        cout << "[ Connected ] ";
    }
    else
    {
        cout << "[ Unconnected ] ";
    }
    //是否上锁
    if (_Drone_state.armed == true)
    {
        cout << "[ Armed ] ";
    }
    else
    {
        cout << "[ DisArmed ] ";
    }
    //是否在地面
    if (_Drone_state.landed == true)
    {
        cout << "[ Ground ] ";
    }
    else
    {
        cout << "[ Air ] ";
    }

    cout << "[ " << _Drone_state.mode<<" ] " <<endl;

    cout << "Position [X Y Z] : " << _Drone_state.position[0] << " [ m ] "<< _Drone_state.position[1]<<" [ m ] "<<_Drone_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << _Drone_state.velocity[0] << " [m/s] "<< _Drone_state.velocity[1]<<" [m/s] "<<_Drone_state.velocity[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << _Drone_state.attitude[0] * 180/M_PI <<" [deg] "<<_Drone_state.attitude[1] * 180/M_PI << " [deg] "<< _Drone_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;

    switch(SwarmCommand.Mode)
    {
        case prometheus_msgs::SwarmCommand::Idle:
            if(SwarmCommand.yaw_ref == 999)
            {
                cout << "Command: [ Idle + Arming + Switching to OFFBOARD mode ] " <<endl;
            }else
            {
                cout << "Command: [ Idle ] " <<endl;
            }
            break;

        case prometheus_msgs::SwarmCommand::Takeoff:
            cout << "Command: [ Takeoff ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Land:
            cout << "Command: [ Land ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Disarm:
            cout << "Command: [ Disarm ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Position_Control:
            if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::One_column)
            {
                cout << "Command: [ Position_Control ] [One_column] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
            {
                cout << "Command: [ Position_Control ] [Triangle] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Square)
            {
                cout << "Command: [ Position_Control ] [Square] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Circular)
            {
                cout << "Command: [ Position_Control ] [Circular] size: " << SwarmCommand.swarm_size <<endl;
            }
            
            formation = formation_utils::get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);

            x = SwarmCommand.position_ref[0] + formation(uav_id-1,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id-1,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id-1,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id-1,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Velocity_Control:
            if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::One_column)
            {
                cout << "Command: [ Velocity_Control ] [One_column] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
            {
                cout << "Command: [ Velocity_Control ] [Triangle] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Square)
            {
                cout << "Command: [ Velocity_Control ] [Square] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Circular)
            {
                cout << "Command: [ Velocity_Control ] [Circular] size: " << SwarmCommand.swarm_size <<endl;
            }

            formation = formation_utils::get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);

            x = SwarmCommand.position_ref[0] + formation(uav_id-1,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id-1,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id-1,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id-1,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Accel_Control:
            cout << "Command: [ Accel_Control ] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::Move:

            if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS)
            {
                cout << "Command: [ Move in XYZ_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            {
                cout << "Command: [ Move in XY_VEL_Z_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::TRAJECTORY)
            {
                cout << "Command: [ Move in TRAJECTORY] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Velocity [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.velocity_ref[2]<<" [m/s] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else
            {
                cout << " wrong sub move mode. " <<endl;
            }
            break;
            
        case prometheus_msgs::SwarmCommand::User_Mode1:
            cout << "Command: [ User_Mode1 ] " <<endl;
            break;
    }
}