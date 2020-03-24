/***************************************************************************************************************************
 * color_line_following.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.1.12
 *
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>
#include <tf/transform_datatypes.h>
//topic 头文件
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/ControlCommand.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;   
Eigen::Vector3f drone_pos;

//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo Detection_raw;          //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    Detection_raw = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_line_following");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    // 来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/target", 10, vision_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

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
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please select one color，1 for black, 2 for blue, 3 for red: "<<endl;
        cin >> start_flag;
    }

    // 起飞
    cout<<"[color_line_following]: "<<"Takeoff to predefined position."<<endl;
    Command_Now.Command_ID = 1;
    while( _DroneState.position[2] < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = 2.0;
        Command_Now.Reference_State.position_ref[1]     = 2.0;
        Command_Now.Reference_State.position_ref[2]     = 2.0;
        Command_Now.Reference_State.yaw_ref             = 0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }


    // 先读取一些飞控的数据
    for(int i=0;i<10;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }


    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position

        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        //command_pub.publish(Command_Now);

        rate.sleep();

    }

    return 0;

}


// void printf_result()
// {
//     cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

//     cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
//     if(is_detected)
//     {
//         cout << "is_detected: ture" <<endl;
//     }else
//     {
//         cout << "is_detected: false" <<endl;
//     }
    
   
    
//     cout << "pos_body_frame: " << pos_body_frame[0] << " [m] "<< pos_body_frame[1] << " [m] "<< pos_body_frame[2] << " [m] "<<endl;

//     cout << "pos_map_frame: " << pos_map_frame[0] << " [m] "<< pos_map_frame[1] << " [m] "<< pos_map_frame[2] << " [m] "<<endl;
//     cout << "ground_truth: " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
//     cout << "Yaw_detect: " << Detection_raw.yaw_error/3.1415926 *180 << " [deg] "<<endl;

//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(GroundTruth.pose.pose.orientation, quat);
 
//     double roll, pitch, yaw;//定义存储r\p\y的容器
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

//     cout << "Yaw_gt: " << yaw/3.1415926 *180 << " [du] "<<endl;

//     cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

//     cout << "pos_des: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;
// }
// void printf_param()
// {
//     cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
//     cout << "Thres_vision : "<< Thres_vision << endl;
//     cout << "distance_thres : "<< distance_thres << endl;
//     cout << "landing_pad_height : "<< landing_pad_height << endl;
//     cout << "kpx_land : "<< kpx_land << endl;
//     cout << "kpy_land : "<< kpy_land << endl;
//     cout << "kpz_land : "<< kpz_land << endl;
//     cout << "start_point_x : "<< start_point_x << endl;
//     cout << "start_point_y : "<< start_point_y << endl;
//     cout << "start_point_z : "<< start_point_z << endl;
// }

