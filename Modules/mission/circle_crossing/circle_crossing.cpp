//ros头文件
#include <ros/ros.h>
#include <iostream>
#include <mission_utils.h>
#include "message_utils.h"

using namespace std;
# define NODE_NAME "circle_crossing"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Detection_result ellipse_det;
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
Eigen::Matrix3f R_Body_to_ENU;
ros::Publisher command_pub,message_pub;
int State_Machine = 0;
float kpx_circle_track,kpy_circle_track,kpz_circle_track;                                                 //控制参数 - 比例参数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void tracking();
void crossing();
void return_start_point();
void detection_result_printf(const struct Detection_result& test);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void ellipse_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr& msg)
{
    ellipse_det.object_name = "circle";
    ellipse_det.Detection_info = *msg;
    ellipse_det.pos_body_frame[0] =   ellipse_det.Detection_info.position[2] + FRONT_CAMERA_OFFSET_X;
    ellipse_det.pos_body_frame[1] = - ellipse_det.Detection_info.position[0] + FRONT_CAMERA_OFFSET_Y;
    ellipse_det.pos_body_frame[2] = - ellipse_det.Detection_info.position[1] + FRONT_CAMERA_OFFSET_Z;

    ellipse_det.pos_body_enu_frame = R_Body_to_ENU * ellipse_det.pos_body_frame;

    if(ellipse_det.Detection_info.detected)
    {
        ellipse_det.num_regain++;
        ellipse_det.num_lost = 0;
    }else
    {
        ellipse_det.num_regain = 0;
        ellipse_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(ellipse_det.num_lost > VISION_THRES)
    {
        ellipse_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(ellipse_det.num_regain > VISION_THRES)
    {
        ellipse_det.is_detected = true;
    }

}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_crossing");
    ros::NodeHandle nh("~");
    
    //【订阅】图像识别结果，返回的结果为相机坐标系
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    ros::Subscriber ellipse_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/ellipse_det", 10, ellipse_det_cb);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message", 10);

    nh.param<float>("kpx_circle_track", kpx_circle_track, 0.1);
    nh.param<float>("kpy_circle_track", kpy_circle_track, 0.1);
    nh.param<float>("kpz_circle_track", kpz_circle_track, 0.1);

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

    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Circle Crossing Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff the drone..."<<endl;
        cin >> start_flag;
    }

    // 起飞
    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;
    while( _DroneState.position[2] < 0.3)
    {
	/** 自动解锁实际飞机比较危险，且没有进行对飞机当前飞行模式状态和解锁状态进行反馈，采用手动解锁并切offboard模式
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Switch to OFFBOARD and arm ....");
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
	*/
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = 0.0;
        Command_Now.Reference_State.position_ref[1]     = 0.0;
        Command_Now.Reference_State.position_ref[2]     = 1.0;
        Command_Now.Reference_State.yaw_ref             = 0.0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    while (ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>Circle Crossing Mission<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
        
        if(State_Machine == 0)
        {
            cout << "Please enter 1 to start the mission ..."<<endl;
            cin >> start_flag;

            if (start_flag == 1)
            {
                State_Machine = 1;
            }else
            {
                State_Machine = 0;
            }
        }else if(State_Machine == 1)
        {
            tracking(); 
            printf_detection_result(ellipse_det);
	    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Tracking the circle...");
        }else if(State_Machine == 2)
        {
            tracking();
            cout << "Crossing the circle..." <<  endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Crossing the circle...");

        }else if(State_Machine == 3)
        {
            tracking();
            cout << "Returning the start point..." <<  endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Returning the start point...");
        }

        //回调
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    return 0;

}

void tracking()
{
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_VEL;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = kpx_circle_track * ( ellipse_det.pos_body_enu_frame[0] - 2.0 );
    Command_Now.Reference_State.velocity_ref[1]     = kpy_circle_track * ellipse_det.pos_body_enu_frame[1];
    Command_Now.Reference_State.velocity_ref[2]     = kpz_circle_track * ellipse_det.pos_body_enu_frame[2];
    Command_Now.Reference_State.yaw_ref             = 0;

    command_pub.publish(Command_Now);   

    if(abs(ellipse_det.pos_body_enu_frame[0]) < 1)
    {
        State_Machine = 2;
    }
}

void crossing()
{
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 2.0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.yaw_ref             = 0;

    command_pub.publish(Command_Now);   

    ros::Duration(5.0).sleep();

    State_Machine = 3;
}

void return_start_point()
{
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0.0;
    Command_Now.Reference_State.position_ref[1]     = 3.0;
    Command_Now.Reference_State.position_ref[2]     = 0.0;
    Command_Now.Reference_State.yaw_ref             = 0.0;

    command_pub.publish(Command_Now);   

    ros::Duration(3.0).sleep();

    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0.0;
    Command_Now.Reference_State.position_ref[1]     = 0.0;
    Command_Now.Reference_State.position_ref[2]     = 1.5;
    Command_Now.Reference_State.yaw_ref             = 0.0;

    command_pub.publish(Command_Now);   

    ros::Duration(5.0).sleep();

    State_Machine = 0;
}

