//无人机追踪目标（此处先直接用目标位置进行追踪）
//无人机吊舱根据视觉算法的反馈调节角度，保证目标在相机中心位置
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include "mission_utils.h"
#include "gimbal_control.h"
#include "message_utils.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "gimbal_control_demo"
#define PI 3.1415926
#define VISION_THRES_TRACKING 100

bool hold_mode;
bool ignore_vision;
Eigen::Vector3d gimbal_att_sp;
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att_rate;
Eigen::Vector3d gimbal_att_rate_deg;
nav_msgs::Odometry GroundTruth;             // 降落板真实位置（仿真中由Gazebo插件提供）
float sight_angle[2];
float desired_yaw;
float kp_track[3];         //控制参数 - 比例参数
float kpyaw_track;
float start_point[3];    // 起始降落位置
Eigen::Vector3d roi_point;
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Matrix3f R_Body_to_ENU,R_camera_to_body;              // 无人机机体系至惯性系转换矩阵

prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
float gimbal_rate;
Eigen::Vector3d mav_pos_;
float distance_to_target;
float integral = 0;
float ki_track;
void printf_result();
void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;

    roi_point[0] = GroundTruth.pose.pose.position.x;
    roi_point[1] = GroundTruth.pose.pose.position.y;
    roi_point[2] = GroundTruth.pose.pose.position.z;
}

int num_regain = 0;
int num_lost = 0;
bool is_detected = false;
Eigen::Vector3d aruco_pos_enu;
prometheus_msgs::ArucoInfo aruco_info;

void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr& msg)
{
    aruco_info = *msg;

    // 暂不考虑无人机姿态的影响
    aruco_pos_enu[0] = mav_pos_[0] - aruco_info.position[1];
    aruco_pos_enu[1] = mav_pos_[1] - aruco_info.position[0];
    // 相机安装在无人机下方10cm处，需减去该偏差
    aruco_pos_enu[2] = mav_pos_[2] - aruco_info.position[2];

    if(aruco_info.detected)
    {
        num_regain++;
        num_lost = 0;
    }else
    {
        num_regain = 0;
        num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(num_lost > VISION_THRES_TRACKING)
    {
        is_detected = false;

        //　丢失后　对sight_angle清零，否则云台会移动
        sight_angle[0] = 0.0;
        sight_angle[1] = 0.0;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(num_regain > 2)
    {
        is_detected = true;
    }

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
    

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);

    mav_pos_ << _DroneState.position[0],_DroneState.position[1],_DroneState.position[2];

}
void gimbal_control_cb(const ros::TimerEvent& e)
{
    // 几个思考的点： 云台控制频率！！！ 设置定时函数，降低云台控制频率
    // 通过什么控制云台
    // 云台角度反馈如何控制无人机速度， 或者是加速度？ 是否需要积分项？
    // 多了姿态反馈后 如何使得与降落版垂直
    // 增加追踪人的场景
    if(ignore_vision)
    {
        Eigen::Vector3d error_vec;
        double distance_2d;
        error_vec = roi_point - mav_pos_;
        distance_2d = std::sqrt(error_vec(0) * error_vec(0) + error_vec(1) * error_vec(1));
        // 理想的吊舱控制情况
        gimbal_att_sp[0] = 0.0;
        gimbal_att_sp[1] = std::atan2(error_vec(2), distance_2d)/PI*180; //pitch
        //desired_yaw = -std::atan2(error_vec(1), error_vec(0))/PI*180;//yaw
        gimbal_att_sp[2] = 0.0;
    }else
    {
        // 使用目标位置的视场角控制云台
        // 品灵吊舱原做法：根据像素来控制云台
        // 传统做法(Jin Ren)：使用视场角误差来反馈，控制吊舱角速度(PD control)
        // 但我们这里只能直接控制云台角度（不能直接用于品灵吊舱的控制），注意sight_angle是误差值
        //　遇到的问题１：由于传进来的sight_angle有噪音，导致云台有一点点抽搐（可以考虑做一个平滑滤波，或者降低控制频率）
        // 云台滚转角不控制
        gimbal_att_sp[0] = 0.0;
        // 云台俯仰角
        gimbal_att_sp[1] = gimbal_att_deg[1] - 0.1 * sight_angle[1]/PI*180; //pitch
        // 云台偏航角 取决于左右夹角
        //gimbal_att_sp[2] = gimbal_att_deg[2] + 0.1 * sight_angle[0]/PI*180; 
        gimbal_att_sp[2] = 0.0;
    }


}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh("~");

    // 节点运行频率： 20hz 
    ros::Rate rate(20.0);
    
    bool moving_target;
    nh.param<bool>("hold_mode", hold_mode, false);
    nh.param<bool>("ignore_vision", ignore_vision, false);
    nh.param<bool>("moving_target", moving_target, false);
    nh.param<float>("gimbal_rate", gimbal_rate, 0.1);
    //追踪控制参数
    nh.param<float>("kpx_track", kp_track[0], 0.1);
    nh.param<float>("kpy_track", kp_track[1], 0.1);
    nh.param<float>("kpz_track", kp_track[2], 0.1);
    nh.param<float>("ki_track", ki_track, 0.02);
    nh.param<float>("kpyaw_track", kpyaw_track, 0.1);
    

    // 初始起飞点
    nh.param<float>("start_point_x", start_point[0], 0.0);
    nh.param<float>("start_point_y", start_point[1], 1.0);
    nh.param<float>("start_point_z", start_point[2], 1.5);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/marker", 10, groundtruth_cb);

    //【订阅】
    ros::Subscriber aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //　吊舱控制timer
    ros::Timer timer = nh.createTimer(ros::Duration(gimbal_rate), gimbal_control_cb);
    
    gimbal_control gimbal_control_;

    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;

    float gimbal_att_sp_deg[3];

    bool sim_mode = true;
    if(sim_mode)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Gimbal Tracking Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
            cin >> start_flag;
        }

        while(ros::ok() && _DroneState.mode != "OFFBOARD")
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
    }

    // 起飞
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Takeoff to predefined position.");

    while( _DroneState.position[2] < 0.5)
    {      
        Command_Now.header.stamp                        = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source                              = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point[0];
        Command_Now.Reference_State.position_ref[1]     = start_point[1];
        Command_Now.Reference_State.position_ref[2]     = start_point[2];
        Command_Now.Reference_State.yaw_ref             = 0.0;
        command_pub.publish(Command_Now);
        cout << "Takeoff to start point..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    // 等待
    ros::Duration(3.0).sleep();

    while(ros::ok())
    {
        //　云台数据打印
        gimbal_att = gimbal_control_.get_gimbal_att();
        gimbal_att_deg = gimbal_att/PI*180;
        cout << "gimbal_att    : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;
        gimbal_att_rate = gimbal_control_.get_gimbal_att_rate();
        gimbal_att_rate_deg = gimbal_att_rate/PI*180;
        cout << "gimbal_att_rate    : " << gimbal_att_rate_deg[0] << " [deg/s] "<< gimbal_att_rate_deg[1] << " [deg/s] "<< gimbal_att_rate_deg[2] << " [deg/s] "<<endl;
        R_camera_to_body = get_rotation_matrix(gimbal_att[0], gimbal_att[1], gimbal_att[2]);

        gimbal_control_.send_mount_control_command(gimbal_att_sp);
        // 无人机追踪目标
        //　但如果是地面目标（或高度确定目标），则应当定高飞行，即XY_VEL_Z_POS
        //　如果是空中目标（高度会变化的目标），应当使用云台角度控制ｚ轴速度，即XYZ_VEL
        if (!hold_mode)
        {
            if(ignore_vision)
            {
                Command_Now.header.stamp                        = ros::Time::now();
                Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
                Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
                Command_Now.source                              = NODE_NAME;
                Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
                //　由于无人机偏航打死，因此　此处使用ENU_FRAME　而非　BODY_FRAME (实际中应当考虑使用机体坐标系进行控制)
                Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
                //float error = GroundTruth.pose.pose.position.x - 2.0 - mav_pos_[0];
                // 策略！！ 策略！！
                // 如：进入指定距离后 控制反馈变小
                // 仍有很大进步空间，但是实际实验可能还要继续调试
                distance_to_target = (roi_point - mav_pos_).norm();
                float error = distance_to_target - 3.0;
                cout << "distance_to_target: " << distance_to_target << " [m] "<<endl;

                integral = integral + error;
                    if (integral > 10.0)
                    {
                        integral = 10;
                    }
                Command_Now.Reference_State.velocity_ref[0]     = kp_track[0] * error + ki_track*integral; 
                    if (Command_Now.Reference_State.velocity_ref[0] <0 )
                    {
                        Command_Now.Reference_State.velocity_ref[0] = 0;
                    }
                    if(moving_target)
                    {
                        Command_Now.Reference_State.velocity_ref[0] = Command_Now.Reference_State.velocity_ref[0] +0.3;
                    }

                // 如果不控制无人机偏航角（即yaw_ref = 0），可根据云台偏航角控制无人机y轴速度
                // y轴速度应当根据视觉解算的目标姿态来调整？ 待定
                Command_Now.Reference_State.velocity_ref[1]     = 0.0;
                // z轴速度取决与当前云台俯仰角度（俯仰角速度） 注意gimbal_att_deg的角度是deg
                // 高度上保持一致，还是保持一个10度俯仰角
                //　如果是高度会变化的目标，应当使用云台角度控制高度
                //　但如果是地面目标（或高度确定目标），则应当定高飞行
                // Command_Now.Reference_State.velocity_ref[2]     = kp_track[2] * (gimbal_att_deg[１] + 2);   
                Command_Now.Reference_State.position_ref[2]     = 2.5;
                // 偏航角 取决于当前云台偏航角
                //　由于仿真云台使用的是与无人机的相对夹角，应此不能控制无人机偏航角或者偏航角速度，需锁定在0度
                // 或者仿真中：不控制云台偏航角，但是仅控制无人机偏航角
                Command_Now.Reference_State.Yaw_Rate_Mode       = false;
                Eigen::Vector3d error_vec;
                error_vec = roi_point - mav_pos_;
                desired_yaw = std::atan2(error_vec(1), error_vec(0));//yaw
                //cout << "desired_yaw: " << desired_yaw/PI*180 << " [deg] "<<endl;
                Command_Now.Reference_State.yaw_ref        = desired_yaw;
                //Command_Now.Reference_State.yaw_rate_ref        = kpyaw_track * (desired_yaw - _DroneState.attitude[2]);
            }else
            {
                // 和降落不同，此处即使丢失也要继续追，不然没法召回目标
                if(is_detected)
                {
                    Command_Now.header.stamp                        = ros::Time::now();
                    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
                    Command_Now.source                              = NODE_NAME;
                    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
                    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
                    // 此处暂时使用真值，实际中应当使用机体坐标系进行控制
                    //float error = GroundTruth.pose.pose.position.x - 2.0 - mav_pos_[0];
                    distance_to_target = (roi_point - mav_pos_).norm();
                    float error = distance_to_target - 3.0;
                    cout << "distance_to_target: " << distance_to_target << " [m] "<<endl;

                    integral = integral + error;
                        if (integral > 10.0)
                        {
                            integral = 10;
                        }
                    Command_Now.Reference_State.velocity_ref[0]     = kp_track[0] * error + ki_track*integral; 
                    if (Command_Now.Reference_State.velocity_ref[0] <0 )
                    {
                        Command_Now.Reference_State.velocity_ref[0] = 0;
                    }
                    if(moving_target)
                    {
                        Command_Now.Reference_State.velocity_ref[0] = Command_Now.Reference_State.velocity_ref[0] +0.3;
                    }


                    // 如果不控制无人机偏航角（即yaw_ref = 0），可根据云台偏航角控制无人机y轴速度
                    Command_Now.Reference_State.velocity_ref[1]     = 0.0;
                    // y轴速度应当根据视觉解算的目标姿态来调整？ 待定
                    // Command_Now.Reference_State.velocity_ref[1]     = - kp_track[1] * gimbal_att_deg[2];
                    // z轴速度取决与当前云台俯仰角度（俯仰角速度） 注意gimbal_att_deg的角度是deg
                    Command_Now.Reference_State.position_ref[2]     = 2.5;
                    // 偏航角 取决于当前云台偏航角
                    //　由于仿真云台使用的是与无人机的相对夹角，应次不能控制无人机偏航角或者偏航角速度，需锁定在0度
                    Command_Now.Reference_State.Yaw_Rate_Mode       = true;
                    Command_Now.Reference_State.yaw_ref        = 0.0;
                    Command_Now.Reference_State.yaw_rate_ref        = - kpyaw_track * sight_angle[0];
                }else
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
                }
            }
            command_pub.publish(Command_Now);
        }


        printf_result();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}

void printf_result()
{
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

    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if( is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    // cout << "Target_pos (camera): " <<  pos_camera_frame[0] << " [m] "<<  pos_camera_frame[1] << " [m] "<<  pos_camera_frame[2] << " [m] "<<endl;
    // cout << "Target_pos (body): " <<  pos_body_frame[0] << " [m] "<<  pos_body_frame[1] << " [m] "<<  pos_body_frame[2] << " [m] "<<endl;
    // cout << "Target_pos (body_enu): " <<  pos_body_enu_frame[0] << " [m] "<<  pos_body_enu_frame[1] << " [m] "<<  pos_body_enu_frame[2] << " [m] "<<endl;
    // cout << "Detection_ENU(pos): " <<  pos_enu_frame[0] << " [m] "<<  pos_enu_frame[1] << " [m] "<<  pos_enu_frame[2] << " [m] "<<endl;
    cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
}

