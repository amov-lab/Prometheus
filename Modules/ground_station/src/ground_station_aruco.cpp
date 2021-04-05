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
prometheus_msgs::MultiArucoInfo multi_aruco_info;

prometheus_msgs::DroneState _DroneState;    // 无人机状态
bool get_drone_pos = false;
Eigen::Vector3d mav_pos_;
Eigen::Vector3d aruco_pos_enu;
Eigen::Matrix3f R_Body_to_ENU,R_camera_to_body;              // 无人机机体系至惯性系转换矩阵
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr& msg)
{
    aruco_info = *msg;

    if(get_drone_pos)
    {
        // 暂不考虑无人机姿态的影响
        aruco_pos_enu[0] = mav_pos_[0] - aruco_info.position[1];
        aruco_pos_enu[1] = mav_pos_[1] - aruco_info.position[0];
        // 相机安装在无人机下方10cm处，需减去该偏差
        aruco_pos_enu[2] = mav_pos_[2] - aruco_info.position[2] - 0.1;
    }
}
void multi_aruco_cb(const prometheus_msgs::MultiArucoInfo::ConstPtr& msg)
{
    multi_aruco_info = *msg;
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    
    get_drone_pos = true;

    mav_pos_ << _DroneState.position[0],_DroneState.position[1],_DroneState.position[2];
}
void printf_info();                                                                       //打印函数
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
    
    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
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
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Ground Station Aruco <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
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

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Aruco Info<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    
    if(aruco_info.detected)
    {
        cout << "Aruco_ID: [" << aruco_info.aruco_num << "]  detected: [yes] " << endl;
        cout << "Pos [camera]: "<< aruco_info.position[0] << " [m] "<< aruco_info.position[1] << " [m] "<< aruco_info.position[2] << " [m] "<<endl;
        cout << "Pos [enu]   : "<< aruco_pos_enu[0]       << " [m] "<< aruco_pos_enu[1]       << " [m] "<< aruco_pos_enu[2]       << " [m] "<<endl;
        // cout << "Att [camera]: "<< aruco_info.position[0] << " [m] "<< aruco_info.position[1] << " [m] "<< aruco_info.position[2] << " [m] "<<endl;
        cout << "Sight Angle : "<< aruco_info.sight_angle[0]/3.14*180 << " [deg] "<< aruco_info.sight_angle[1]/3.14*180 << " [deg] " <<endl;
    }else
    {
        cout << "Aruco_ID: [" << aruco_info.aruco_num << "]  detected: [no] " << endl;
    }
    
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Multi Aruco Info<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    for (int i=0; i<multi_aruco_info.num_arucos; i++)
    {
        cout << "Aruco_ID: [" << multi_aruco_info.aruco_infos[i].aruco_num << "]  detected: [yes] " << endl;
        
        if(get_drone_pos)
        {
            Eigen::Vector3d multi_aruco_pos_enu;
            // 暂不考虑无人机姿态的影响
            multi_aruco_pos_enu[0] = mav_pos_[0] - multi_aruco_info.aruco_infos[i].position[1];
            multi_aruco_pos_enu[1] = mav_pos_[1] - multi_aruco_info.aruco_infos[i].position[0];
            // 相机安装在无人机下方10cm处，需减去该偏差
            multi_aruco_pos_enu[2] = mav_pos_[2] - multi_aruco_info.aruco_infos[i].position[2] - 0.1;
            cout << "Pos [enu]   : "<< multi_aruco_pos_enu[0]       << " [m] "<< multi_aruco_pos_enu[1]       << " [m] "<< multi_aruco_pos_enu[2]       << " [m] "<<endl;
        }
    }

}
