/***************************************************************************************************************************
* ground_station_info.cpp
*
* Author: Qyp
*
* Update Time: 2020.12.15
*
* Introduction:  Prometheus 数据信息地面站 
*         1. 根据不同任务选择打印不同消息
*         2. 一般配合SSH使用,本程序运行于地面电脑
*         3. 带宽不足时,请降低本程序运行频率
***************************************************************************************************************************/
//头文件
#include <ros/ros.h>
#include <prometheus_station_utils.h>

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
float refresh_time;
int mission_type;
int control_type;

prometheus_msgs::DroneState _DroneState;
prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::AttitudeReference _AttitudeReference;

prometheus_msgs::DetectionInfo detection_info;

geometry_msgs::PoseStamped ref_pose;
Eigen::Quaterniond q_fcu_target;
Eigen::Vector3d euler_fcu_target;
float Thrust_target;
//Target pos of the drone [from fcu]
Eigen::Vector3d pos_drone_fcu_target;
//Target vel of the drone [from fcu]
Eigen::Vector3d vel_drone_fcu_target;
//Target accel of the drone [from fcu]
Eigen::Vector3d accel_drone_fcu_target;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    euler_fcu_target = prometheus_station_utils::quaternion_to_euler(q_fcu_target);

    Thrust_target = - msg->thrust;
}
void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
}
void log_control_cb(const prometheus_msgs::LogMessageControl::ConstPtr& msg)
{
    control_type = msg->control_type;
    _DroneState = msg->Drone_State;
    Command_Now = msg->Control_Command;
    _AttitudeReference = msg->Attitude_Reference;
    ref_pose = msg->ref_pose;
}
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    detection_info = *msg;

    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 此处未考虑相机安装误差
    detection_info.position[0] = - msg->position[1];
    detection_info.position[1] = - msg->position[0];
    detection_info.position[2] = - msg->position[2];

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param<float>("refresh_time", refresh_time, 1.0);
    // 根据任务类型 订阅不同话题,打印不同话题
    // 
    nh.param<int>("mission_type", mission_type, 0);


    // 【订阅】prometheus_control模块回传的消息
    ros::Subscriber log_control_sub = nh.subscribe<prometheus_msgs::LogMessageControl>("/prometheus/log/control", 10, log_control_cb);
    
    // 【订阅】飞控回传
    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, att_target_cb);
    
    // 【订阅】 本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为POSITION_TARGET_LOCAL_NED, 对应的飞控中的uORB消息为vehicle_local_position_setpoint.msg
    ros::Subscriber position_target_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, pos_target_cb);

    if(mission_type == 1)
    {
        ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/ellipse_det", 10, landpad_det_cb);
    }
    
    // 频率
    float hz = 1.0 / refresh_time;
    ros::Rate rate(hz);


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

    // 【打印】无人机状态,包括位置,速度等数据信息
    prometheus_station_utils::prinft_drone_state(_DroneState);

    // 【打印】来自上层的控制指令
    prometheus_station_utils::printf_command_control(Command_Now);

    // 【打印】控制模块消息
    if(control_type == 1)
    {
        //打印期望位姿
        prometheus_station_utils::prinft_ref_pose(ref_pose);
        prometheus_station_utils::prinft_attitude_reference(_AttitudeReference);

        cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        
        cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg]  "<<euler_fcu_target[1] * 180/M_PI << " [deg]  "<< euler_fcu_target[2] * 180/M_PI<<" [deg]  "<<endl;
        
        cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl;
    }else if(control_type == 0)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>> Target Info from PX4 <<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_target [X Y Z] : " << pos_drone_fcu_target[0] << " [ m ] "<< pos_drone_fcu_target[1]<<" [ m ] "<<pos_drone_fcu_target[2]<<" [ m ] "<<endl;
        cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
        // cout << "Acc_target [X Y Z] : " << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;
    }

    // 【打印】视觉模块消息
    if(mission_type == 1)
    {
        // 降落任务
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        if(detection_info.detected)
        {
            cout << "is_detected: ture" <<endl;
        }else
        {
            cout << "is_detected: false" <<endl;
        }
        
        cout << "detection_result (body): " << detection_info.position[0] << " [m] "<< detection_info.position[1] << " [m] "<< detection_info.position[2] << " [m] "<<endl;
    }





}
