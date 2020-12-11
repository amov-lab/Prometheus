
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Range.h>
#include <mission_utils.h>

#include "message_utils.h"

using namespace std;
using namespace Eigen;

#define LANDPAD_HEIGHT 0.01
#define NODE_NAME "pad_tracking"
#define DEBUG_MODE 1 // DEBUG_MODE设置为1用于调试模式,不影响功能,但会打印更多信息
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;   
Eigen::Matrix3f R_Body_to_ENU;
//---------------------------------------Vision---------------------------------------------
Detection_result landpad_det;
float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数
float start_point_x,start_point_y,start_point_z;

Eigen::Vector3f camera_offset;
//-----　laser
sensor_msgs::Range tfmini;
//---------------------------------------Track---------------------------------------------
float distance_to_pad;
float arm_height_to_ground;
float arm_distance_to_pad;
nav_msgs::Odometry GroundTruth;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 相机安装误差 在mission_utils.h中设置
    landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + DOWN_CAMERA_OFFSET_X;
    landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + DOWN_CAMERA_OFFSET_Y;
    landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + DOWN_CAMERA_OFFSET_Z;

    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;

    // 若已知降落板高度，则无需使用深度信息。
    // landpad_det.pos_body_enu_frame[2] = LANDPAD_HEIGHT - _DroneState.position[2];

    // 机体惯性系 -> 惯性系
    landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
    landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
    // 使用TFmini测得高度
    landpad_det.pos_enu_frame[2] = tfmini.range;

    landpad_det.att_enu_frame[2] = 0.0;

    if(landpad_det.Detection_info.detected)
    {
        landpad_det.num_regain++;
        landpad_det.num_lost = 0;
    }else
    {
        landpad_det.num_regain = 0;
        landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(landpad_det.num_lost > VISION_THRES)
    {
        landpad_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(landpad_det.num_regain > VISION_THRES)
    {
        landpad_det.is_detected = true;
    }

}
void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}
void tfmini_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    tfmini = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pad_tracking_station");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/ellipse_det", 10, landpad_det_cb);

    //【订阅】无人机高度(由tfmini测量得到)
    ros::Subscriber tfmini_sub = nh.subscribe<sensor_msgs::Range>("/prometheus/tfmini_range", 10, tfmini_cb);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruth_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //强制上锁高度
    nh.param<float>("arm_height_to_ground", arm_height_to_ground, 0.2);
    //强制上锁距离
    nh.param<float>("arm_distance_to_pad", arm_distance_to_pad, 0.2);

    //追踪控制参数
    nh.param<float>("kpx_land", kpx_land, 0.1);
    nh.param<float>("kpy_land", kpy_land, 0.1);
    nh.param<float>("kpz_land", kpz_land, 0.1);

    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
    nh.param<float>("start_point_z", start_point_z, 2.0);

#if DEBUG_MODE == 1
    //打印现实检查参数
    printf_param();
#endif

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


    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;

    if(sim_mode == true)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
            cin >> start_flag;
        }

        while(_DroneState.mode != "OFFBOARD")
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.yaw_ref = 999;
            command_pub.publish(Command_Now);   
            cout << "Switch to OFFBOARD and arm ..."<<endl;
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }else
    {
        while(_DroneState.mode != "OFFBOARD")
        {
            cout << "Waiting for the offboard mode"<<endl;
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
    }
    
    // 起飞
    cout<<"[autonomous_landing]: "<<"Takeoff to predefined position."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Takeoff to predefined position.");

    while( _DroneState.position[2] < 0.5)
    {      
        Command_Now.header.stamp                        = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source                              = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point_x;
        Command_Now.Reference_State.position_ref[1]     = start_point_y;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = 0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    // 等待
    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        printf_result();

        distance_to_pad = landpad_det.pos_body_enu_frame.norm();
        //　达到降落距离，上锁降落
        if(distance_to_pad < arm_distance_to_pad)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
            cout <<"[autonomous_landing]: Catched the Landing Pad, distance_to_pad : "<< distance_to_pad << " [m] " << endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Catched the Landing Pad.");
        }
        //　达到最低高度，上锁降落
        else if(abs(landpad_det.pos_body_enu_frame[2]) < arm_height_to_ground)
        {
            cout <<"[autonomous_landing]: Reach the lowest height. "<< endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Reach the lowest height.");
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
        }
        //　丢失降落板，任务失败，悬停
        else if(!landpad_det.is_detected)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            cout <<"[autonomous_landing]: Lost the Landing Pad. "<< endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Lost the Landing Pad.");
        }
        //　执行降落板追踪
        else
        {
            cout <<"[autonomous_landing]: Tracking the Landing Pad, distance_to_pad : "<< distance_to_pad << " [m] " << endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Tracking the Landing Pad.");

            // 机体系速度控制
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;   //xy velocity z position

            Eigen::Vector3f vel_command;
            vel_command[0] = kpx_land * landpad_det.pos_body_frame[0] ;
            vel_command[1] = kpy_land * landpad_det.pos_body_frame[1];
            vel_command[2] = kpz_land * landpad_det.pos_body_frame[2];

            for (int i=0; i<3; i++)
            {
                Command_Now.Reference_State.velocity_ref[i] = vel_command[i];
            }


            Command_Now.Reference_State.yaw_ref             = 0.0;
            
        }

        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;

        if (debug_mode == 0)
        {
            command_pub.publish(Command_Now);
        }
        

        rate.sleep();

    }

    return 0;

}


void printf_result()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(landpad_det.is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    cout << "Detection_raw(pos): " << landpad_det.pos_body_frame[0] << " [m] "<< landpad_det.pos_body_frame[1] << " [m] "<< landpad_det.pos_body_frame[2] << " [m] "<<endl;
    cout << "Detection_raw(yaw): " << landpad_det.Detection_info.yaw_error/3.1415926 *180 << " [deg] "<<endl;


    if (debug_mode == 1)
    {
        cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
        cout << "Detection_ENU(pos): " << landpad_det.pos_enu_frame[0] << " [m] "<< landpad_det.pos_enu_frame[1] << " [m] "<< landpad_det.pos_enu_frame[2] << " [m] "<<endl;
        cout << "Detection_ENU(yaw): " << landpad_det.att_enu_frame[2]/3.1415926 *180 << " [deg] "<<endl;
    }

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "pos_des: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;

    cout << "vel_cmd: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "arm_distance_to_pad : "<< arm_distance_to_pad << endl;
    cout << "arm_height_to_ground : "<< arm_height_to_ground << endl;
    cout << "kpx_land : "<< kpx_land << endl;
    cout << "kpy_land : "<< kpy_land << endl;
    cout << "kpz_land : "<< kpz_land << endl;
    cout << "start_point_x : "<< start_point_x << endl;
    cout << "start_point_y : "<< start_point_y << endl;
    cout << "start_point_z : "<< start_point_z << endl;
}

