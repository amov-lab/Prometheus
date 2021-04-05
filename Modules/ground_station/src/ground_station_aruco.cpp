//头文件
#include <ros/ros.h>
#include <prometheus_station_utils.h>
#include "control_common.h"
//msg 头文件
#include <prometheus_msgs/ArucoInfo.h>
#include <prometheus_msgs/MultiArucoInfo.h>

using namespace std;
//---------------------------------------相关参数-----------------------------------------------
float refresh_time;
int mission_type;

prometheus_msgs::ArucoInfo aruco_info;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr& msg)
{
    aruco_info = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station_aruco");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param<float>("refresh_time", refresh_time, 1.0);
    // 根据任务类型 订阅不同话题,打印不同话题
    nh.param<int>("mission_type", mission_type, 0);

    // 【订阅】
    ros::Subscriber aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);
    
    // 【订阅】
    ros::Subscriber multi_aruco_sub = nh.subscribe<prometheus_msgs::MultiArucoInfo>("/prometheus/object_detection/multi_aruco_det", 10, multi_aruco_cb);
    
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
    if(control_type == PX4_POS_CONTROLLER)
    {
        //打印期望位姿
        prometheus_station_utils::prinft_ref_pose(ref_pose);
        prometheus_station_utils::prinft_attitude_reference(_AttitudeReference);

        cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        
        cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg]  "<<euler_fcu_target[1] * 180/M_PI << " [deg]  "<< euler_fcu_target[2] * 180/M_PI<<" [deg]  "<<endl;
        
        cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl;
    }else if(control_type == PX4_SENDER)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>> Target Info from PX4 <<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_target [X Y Z] : " << pos_drone_fcu_target[0] << " [ m ] "<< pos_drone_fcu_target[1]<<" [ m ] "<<pos_drone_fcu_target[2]<<" [ m ] "<<endl;
        cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
        // cout << "Acc_target [X Y Z] : " << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;
    }else if(control_type == PX4_GEO_CONTROLLER)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        
        cout << "Bodyrate_target [R P Y] : " << bodyrate_fcu_target[0] * 180/M_PI <<" [deg/s]  "<<bodyrate_fcu_target[1] * 180/M_PI << " [deg/s]  "<< bodyrate_fcu_target[2] * 180/M_PI<<" [deg/s]  "<<endl;
        
        cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl; 
    }

    if(Command_Now.Mode == prometheus_msgs::ControlCommand::Move)
    {
        
        
        //Only for TRAJECTORY tracking
        if(Command_Now.Reference_State.Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
        {
            cout <<">>>>>>>>>>>>>>>>>>>>>>>> Tracking Error <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

            static Eigen::Vector3d tracking_error;
            tracking_error = prometheus_station_utils::tracking_error(_DroneState, Command_Now);
            cout << "Pos_error [m]: " << tracking_error[0] <<endl;
            cout << "Vel_error [m/s]: " << tracking_error[1] <<endl;
        }
        
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

    if(gimbal_enable)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Gimbal State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Gimbal_att    : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;
    }





}
