//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>


#include <state_from_mavros.h>
#include <command_to_mavros.h>

#include <prometheus_control_utils.h>
#include <OptiTrackFeedBackRigidBody.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/TrajectoryPoint.h>
#include <prometheus_msgs/AttitudeReference.h>

#include <prometheus_msgs/Trajectory.h>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>

#include <bitset>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/GroundStation.h>


using namespace std;
//---------------------------------------相关参数-----------------------------------------------
prometheus_msgs::GroundStation _GroundStation;

Eigen::Vector3d pos_drone_mocap;                          //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap;                              //无人机当前姿态 (vicon)
rigidbody_state UAVstate;
Eigen::Quaterniond q_fcu_target;
Eigen::Vector3d euler_fcu_target;
float Thrust_target;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void log_cb(const prometheus_msgs::GroundStation::ConstPtr &msg)
{
    _GroundStation = *msg;
}

void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    euler_fcu_target = quaternion_to_euler(q_fcu_target);

    Thrust_target = msg->thrust;
}

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 ENU系
    int optitrack_frame = 0; //Frame convention 0: Z-up -- 1: Y-up

    if(optitrack_frame == 0)
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
    else
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(-msg->pose.position.x,msg->pose.position.z,msg->pose.position.y);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
    }

    // Transform the Quaternion to Euler Angles
    Euler_mocap = quaternion_to_euler(q_mocap);

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV/pose", 10, optitrack_cb);

    ros::Subscriber log_sub = nh.subscribe<prometheus_msgs::GroundStation>("/prometheus/GroundStation", 10, log_cb);

    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10,att_target_cb);

    // 频率
    ros::Rate rate(10.0);

    OptiTrackFeedBackRigidBody UAV("/vrpn_client_node/UAV/pose",nh,3,3);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //利用OptiTrackFeedBackRigidBody类获取optitrack的数据
        //UAV.GetOptiTrackState();

        UAV.RosWhileLoopRun();
        UAV.GetState(UAVstate);

        //打印
        printf_info();
        rate.sleep();
    }

    return 0;

}

void printf_info()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Ground Station  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    prometheus_control_utils::prinft_drone_state(_GroundStation.Drone_State);

    prometheus_control_utils::printf_command_control(_GroundStation.Control_Command);

    prometheus_control_utils::prinft_attitude_reference(_GroundStation.Attitude_Reference);


    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Control Output  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    
    //cout << "u_l [X Y Z]  : " << _GroundStation.Control_Output.u_l[0] << " [ ] "<< _GroundStation.Control_Output.u_l[1] <<" [ ] "<< _GroundStation.Control_Output.u_l[2] <<" [ ] "<<endl;
    
    //cout << "u_d [X Y Z]  : " << _GroundStation.Control_Output.u_d[0] << " [ ] "<< _GroundStation.Control_Output.u_d[1] <<" [ ] "<< _GroundStation.Control_Output.u_d[2] <<" [ ] "<<endl;
    //cout << "NE  [X Y Z]  : " << _GroundStation.Control_Output.NE[0] << " [ ] "<< _GroundStation.Control_Output.NE[1] <<" [ ] "<< _GroundStation.Control_Output.NE[2] <<" [ ] "<<endl;

    cout << "Thrust  [X Y Z]  : " << _GroundStation.Control_Output.Thrust[0] << " [ ] "<< _GroundStation.Control_Output.Thrust[1] <<" [ ] "<< _GroundStation.Control_Output.Thrust[2] <<" [ ] "<<endl;

    cout << "Throttle  [X Y Z]  : " << _GroundStation.Control_Output.Throttle[0] << " [ ] "<< _GroundStation.Control_Output.Throttle[1] <<" [ ] "<< _GroundStation.Control_Output.Throttle[2] <<" [ ] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    
    cout << "Pos_vicon [X Y Z]  : " << pos_drone_mocap[0] << " [ m ] "<< pos_drone_mocap[1] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
    
    cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg]  "<<euler_fcu_target[1] * 180/M_PI << " [deg]  "<< euler_fcu_target[2] * 180/M_PI<<" [deg]  "<<endl;
    
    cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl;

    //cout <<">>>>>>>>>>>>>>>>>>>>>>>>Error Info [ Longhao ]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //cout << "Error_pos      : " << UAVstate.Position[0] - _GroundStation.Drone_State.position[0] << " [ m ] "<< UAVstate.Position[1] - _GroundStation.Drone_State.position[1]<<" [ m ] "<< UAVstate.Position[2] - _GroundStation.Drone_State.position[2]<<" [ m ] "<<endl;
    //cout << "Error_vel      : " << UAVstate.V_I[0] - _GroundStation.Drone_State.velocity[0] << " [m/s] "<< UAVstate.V_I[1] - _GroundStation.Drone_State.velocity[1]<<" [m/s] "<< UAVstate.V_I[2] - _GroundStation.Drone_State.velocity[2]<<" [m/s] "<<endl;
    //cout << "Error_att      : " << UAVstate.Euler[0]*57.3 - _GroundStation.Drone_State.attitude[0]*57.3 << " [deg] "<< UAVstate.Euler[1]*57.3 - _GroundStation.Drone_State.attitude[1]*57.3<<" [deg] "<< UAVstate.Euler[2]*57.3 - _GroundStation.Drone_State.attitude[2]*57.3<<" [deg] "<<endl;
    //cout << "Error_att_rate : " << UAVstate.Omega_BI[0]*57.3 - _GroundStation.Drone_State.attitude_rate[0]*57.3 << " [deg] "<< UAVstate.Omega_BI[1]*57.3 - _GroundStation.Drone_State.attitude_rate[1]*57.3<<" [deg] "<< UAVstate.Omega_BI[2]*57.3 - _GroundStation.Drone_State.attitude_rate[2]*57.3<<" [deg] "<<endl;

}
