//头文件
#include <ros/ros.h>
#include <prometheus_control_utils.h>


//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;
//---------------------------------------相关参数-----------------------------------------------
prometheus_msgs::DroneState _DroneState;
prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::AttitudeReference _AttitudeReference;
geometry_msgs::PoseStamped ref_pose;
Eigen::Quaterniond q_fcu_target;
Eigen::Vector3d euler_fcu_target;
float Thrust_target;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    euler_fcu_target = quaternion_to_euler(q_fcu_target);

    Thrust_target = msg->thrust;
}

void log_cb(const prometheus_msgs::LogMessage::ConstPtr& msg)
{
    _DroneState = msg->Drone_State;
    Command_Now = msg->Control_Command;
    _AttitudeReference = msg->Attitude_Reference;
}

void ref_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ref_pose = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    ros::Subscriber log_message_sub = nh.subscribe<prometheus_msgs::LogMessage>("/prometheus/topic_for_log", 10, log_cb);
    
    ros::Subscriber ref_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/control/ref_pose_rviz", 10, ref_pose_cb);
    // 【订阅】飞控回传
    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10,att_target_cb);
    // 频率
    ros::Rate rate(1.0);


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

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
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    //打印无人机状态,包括位置,速度等数据信息
    prometheus_control_utils::prinft_drone_state(_DroneState);

    //打印来自上层的控制指令
    prometheus_control_utils::printf_command_control(Command_Now);

    //打印期望位姿
    prometheus_control_utils::prinft_ref_pose(ref_pose);
    prometheus_control_utils::prinft_attitude_reference(_AttitudeReference);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    
    cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg]  "<<euler_fcu_target[1] * 180/M_PI << " [deg]  "<< euler_fcu_target[2] * 180/M_PI<<" [deg]  "<<endl;
    
    cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl;

}
