#include <ros/ros.h>
#include <sstream>
#include <Eigen/Eigen>
#include <iostream>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/TargetsInFrame.h>
#include <prometheus_msgs/Target.h>
#include <prometheus_msgs/ROI.h>
#include <mission_utils.h>
#include "printf_utils.h"
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;
#define NODE_NAME "prosim_aruco_tracking"
// 创建无人机相关数据变量
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state; 
prometheus_msgs::UAVCommand uav_command; 

// 相机系，x:右方为正，y：下方为正，z：前方为正
// 机体系，x:前方为正，y：左方为正，z：上方为正
// 惯性系（enu），x:东方为正，y：北方为正，z：上方为正

// 目标位置,相机系
prometheus_msgs::Target det_res; 

// 存储各项数据的矩阵
Eigen::Vector3f drone_pos; // 机体坐标,enu系
Eigen::Vector3f pos_body_frame; // 目标相对机体的坐标,相机系
Eigen::Vector3f pos_body_enu_frame; // 目标相对机体的坐标,enu系
Eigen::Vector3f camera_offset; // 相机相对无人机的坐标,相机系
Eigen::Vector3f tracking_delta; // 跟踪距离,相机系
// Eigen::Vector3d car_pose_;

float kpx_track, kpy_track, kpz_track; // 跟踪速度比例系数,相机系
bool is_detected = false;              // 是否检测到目标标志
bool is_detected_move = true;              // 是否检测到目标标志

int num_count_vision_lost = 0;         // 视觉丢失计数器
int num_count_vision_regain = 0;       // 视觉丢失计数器
int Thres_vision = 0; // 视觉丢失计数器阈值

int uav_id;// 无人机id
float distance_to_setpoint; // 相对原点距离
prometheus_msgs::TargetsInFrame det; // 单张图像的检测信息
// 目标二维码id
int goto_id = 1;

// 全局变量或在合适的范围定义
const int lost_count_threshold = 25;  // 丢失目标计数阈值（根据频率调节）
const double move_duration = 1.0; // 在丢失后移动持续时间（秒）
const float velocity_smooth_alpha = 0.2; // 平滑系数，0~1，越小越平滑
static float smoothed_velocity[3] = {0.0f, 0.0f, 0.0f};// 用于保存平滑后的速度
static float diff_vel = 0.5;//速度波动，用来判断指数加权平均的平滑处理

bool tracking_state = false;
int lost_count = 0;
bool has_moved_after_lost = false;
ros::Time move_start_time;
float last_tracked_velocity[3] = {0.0, 0.0, 0.0};

void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    drone_pos[0] = uav_state.position[0];
    drone_pos[1] = uav_state.position[1];
    drone_pos[2] = uav_state.position[2];
}

void VisionCb(const prometheus_msgs::TargetsInFrame::ConstPtr &msg)
{
    // printf("tracking state %d\n", msg->tracking_state);
    tracking_state = msg->tracking_state;
    det_res.mode = false;
    for(auto &target : msg->targets)
    {
        if(/*target.tracked_id != goto_id && */target.mode == false)
            continue;
        // std::cout << "target aruco" << std::endl;
        det_res = target;
    }
    // det_res = *msg;
    pos_body_frame[0] = det_res.pz + camera_offset[0];
    pos_body_frame[1] = -det_res.px + camera_offset[1];
    pos_body_frame[2] = -det_res.py + camera_offset[2];
    // pos_body_frame[0] = det_res.px
    
    Eigen::Matrix3f R_Body_to_ENU;

    R_Body_to_ENU = get_rotation_matrix(uav_state.attitude[0], uav_state.attitude[1], uav_state.attitude[2]);

    pos_body_enu_frame = R_Body_to_ENU * pos_body_frame;


    if(det_res.score == 1)
    {
        num_count_vision_regain++;
        num_count_vision_lost = 0;
    }else
    {
        num_count_vision_regain = 0;
        num_count_vision_lost++;
    }
    // 当连续一段时间无法检测到目标时，认定目标丢失
    if (num_count_vision_lost > Thres_vision)
    {
        is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if (num_count_vision_regain > Thres_vision)
    {
        is_detected = true;
    }
}
inline bool checkInput(int &goto_id)
{
    std::cout << "Please input the number(1~20) for that point to land on, input 0 exit program" << std::endl;
    while (true)
    {
        try
        {
            std::cout << " > ";
            std::cin >> goto_id;
            if (goto_id == 0)
            {
                return false;
            }
            break;
        }
        catch (const std::exception &e)
        {
        }
        std::cout << "Input number not in the range 1 to 20, re-input" << std::endl;
    }
    return true;
}


inline float clamp(float value, float max)
{
    max = std::abs(max);
    return std::fmin(std::fmax(value,-max),max);
}

int main(int argc, char **argv)
{
    // ROS初始化,设定节点名
    ros::init(argc, argv, NODE_NAME);
    // 创建句柄
    ros::NodeHandle nh("~");
    // 循环频率设置为25HZ
    ros::Rate rate(20);

    // 从参数服务器接收参数
    // 视觉丢失次数阈值
    nh.getParam("Thres_vision", Thres_vision);

    // 追踪的前后间隔
    nh.getParam("tracking_delta_x", tracking_delta[0]);
    nh.getParam("tracking_delta_y", tracking_delta[1]);
    nh.getParam("tracking_delta_z", tracking_delta[2]);

    // 相机相对无人机的坐标
    nh.getParam("camera_offset_x", camera_offset[0]);
    nh.getParam("camera_offset_y", camera_offset[1]);
    nh.getParam("camera_offset_z", camera_offset[2]);

    // 追踪控制参数
    nh.getParam("kpx_track", kpx_track);
    nh.getParam("kpy_track", kpy_track);
    nh.getParam("kpz_track", kpz_track);

    nh.getParam("uav_id", uav_id);

    printf("uav_id  %d", uav_id);
    float max_velocity = 3;

    // 获取 无人机ENU下的位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state", 10, droneStateCb);
    // 获取视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/target", 10, VisionCb);
    // 【发布】发送给prometheus_uav_control的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 10 );
    // 获取遥控器状态
    ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(uav_id) + "/prometheus/control_state", 10, [&](const prometheus_msgs::UAVControlState::ConstPtr &msg) -> void
                                                                                      { uav_control_state = *msg; });
     // 1.kalman filter setup
     const int stateNum      =6;                                      //状态值6×1向量(x,y,z,△x,△y,△z)
     const int measureNum    =3;                                    //测量值3×1向量(x,y,z)	
     KalmanFilter KF(stateNum, measureNum, 3);	
     KF.transitionMatrix     = (cv::Mat_<float>(6, 6) <<  
         1, 0, 0, 1, 0, 0,  
         0, 1, 0, 0, 1, 0,  
         0, 0, 1, 0, 0, 1,  
         0, 0, 0, 1, 0, 0,  
         0, 0, 0, 0, 1, 0,  
         0, 0, 0, 0, 0, 1); // 转移矩阵  
     setIdentity(KF.measurementMatrix);                                             //测量矩阵H
     setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
     setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
     setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
     //rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
     Mat statenum        = Mat::zeros(stateNum, 1, CV_32F);
     Mat measurement     = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义   
 
     double distance     = 0;// 估计距离
    // 根据设定跟踪距离计算应该给于无人机的速度
    // 速度vel是机体系坐标,det_res为相机系下目标相对相机的坐标, 
    float x_vel                 = 0;
    float y_vel                 = 0;
    float z_vel                 = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if(uav_control_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }
        
        if (tracking_state == 0)
        {
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            PCOUT(-1, GREEN, "No tracking, hovering...\n");
        }else
        {
            // 主循环内逻辑
            if (det_res.mode == false)  // 未识别到目标
            {
                lost_count++;

                if (lost_count > lost_count_threshold ) //先移动再悬停
                {
                    // 丢失但未达到阈值，移动
                    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                    PCOUT(-1, GREEN, "Target lost, hovering...\n");
                    has_moved_after_lost = false;
                }
                else
                {
                    // 丢失时间小于阈值，执行移动
                    if (!has_moved_after_lost)
                    {
                        move_start_time = ros::Time::now();
                        has_moved_after_lost = true;
                        PCOUT(-1, GREEN, "Target lost long time, start moving...\n");
                    }

                    ros::Duration elapsed = ros::Time::now() - move_start_time;

                    // 在移动阶段，如果目标突然恢复，立即切换到目标跟踪，丝滑恢复
                    if (det_res.mode == true)  // 目标恢复检测
                    {
                        lost_count = 0;
                        has_moved_after_lost = false;
                        PCOUT(-1, GREEN, "Target recovered during move, resuming tracking...\n");
                        
                        // 这里可以跳过后续代码，转入目标跟踪代码部分
                        // 方便起见，在主循环外统一处理，或者在这里直接return
                    }
                    else if (elapsed.toSec() < move_duration)
                    {
                        // 持续移动，使用之前记录的速度
                        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                        uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
                        uav_command.yaw_ref = 0;

                        uav_command.velocity_ref[0] = last_tracked_velocity[0];
                        uav_command.velocity_ref[1] = last_tracked_velocity[1];
                        uav_command.velocity_ref[2] = last_tracked_velocity[2];

                        PCOUT(-1, GREEN, "Moving after lost with last velocity...\n");
                    }
                    else
                    {
                        // 移动完成，目标未恢复，悬停
                        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                        PCOUT(-1, GREEN, "Moved after lost, target still lost, hovering.\n");
                    }
                }
            }
            else
            {
                // 识别到目标，复位状态
                lost_count = 0;
                has_moved_after_lost = false;

                // 2.kalman prediction
                Mat prediction = KF.predict();

                float measurementX = det_res.px;
                float measurementY = det_res.py;
                float measurementZ = det_res.pz;
                measurement.at<float>(0) = measurementX;
                measurement.at<float>(1) = measurementY;
                measurement.at<float>(2) = measurementZ;
                KF.correct(measurement);

                double last_distance = distance;
                distance = sqrt(pow(KF.statePost.at<float>(0), 2) + pow(KF.statePost.at<float>(1), 2) + pow(KF.statePost.at<float>(2), 2));

                if (std::abs(distance - last_distance) > 7)
                {
                    printf("waiting for cov\n");
                    rate.sleep();
                    continue;
                }

                float px = prediction.at<float>(0);
                float py = prediction.at<float>(1);
                float pz = prediction.at<float>(2);

                // 计算当前速度
                if (fabs(tracking_delta[2] - pz) < 1)
                    x_vel = kpz_track * pow((pz - tracking_delta[2]), 2) * fabs(pz - tracking_delta[2]) / (pz - tracking_delta[2]);
                else
                    x_vel = kpz_track * (pz - tracking_delta[2]);

                if (fabs(tracking_delta[0] - px) < 1)
                    y_vel = -kpx_track * pow((px - tracking_delta[0]), 2) * fabs(px - tracking_delta[0]) / (px - tracking_delta[0]);
                else
                    y_vel = -kpx_track * (px - tracking_delta[0]);

                if (fabs(tracking_delta[1] - uav_state.position[2]) < 0.3)
                    z_vel = 0;
                else
                    z_vel = -kpy_track * (uav_state.position[2] - tracking_delta[1]) / 3;

                // 限幅
                float target_velocity[3];
                target_velocity[0] = clamp(x_vel, max_velocity);
                target_velocity[1] = clamp(y_vel, max_velocity);
                target_velocity[2] = clamp(z_vel, max_velocity);

                // 平滑处理：用简单的低通滤波（指数加权平均）
                for (int i = 0; i < 3; ++i)
                {
                    // 计算速度差，如果差距过大，可以适当减小alpha使更平滑
                    float diff = std::abs(target_velocity[i] - smoothed_velocity[i]);

                    // 你可以根据实际需求调整diff阈值和alpha
                    if (diff > diff_vel) // 如果差值大于阈值，说明突变，减小alpha提高平滑力度
                    {
                        smoothed_velocity[i] = smoothed_velocity[i] + velocity_smooth_alpha * 0.3f * (target_velocity[i] - smoothed_velocity[i]);
                    }
                    else
                    {
                        // 差值小，正常平滑
                        smoothed_velocity[i] = smoothed_velocity[i] + velocity_smooth_alpha * (target_velocity[i] - smoothed_velocity[i]);
                    }
                }

                // 赋值给输出速度
                last_tracked_velocity[0] = smoothed_velocity[0];
                last_tracked_velocity[1] = smoothed_velocity[1];
                last_tracked_velocity[2] = smoothed_velocity[2];

                // 发送速度命令
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
                uav_command.yaw_ref = 0;

                uav_command.velocity_ref[0] = last_tracked_velocity[0];
                uav_command.velocity_ref[1] = last_tracked_velocity[1];
                uav_command.velocity_ref[2] = last_tracked_velocity[2];

                PCOUT(-1, GREEN, "Tracking target, moving...\n");
            } 
        }
        
        
        // Publish
        uav_command.header.stamp = ros::Time::now();
        uav_command.Command_ID = uav_command.Command_ID + 1;
        if (uav_command.Command_ID < 10)
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        command_pub.publish(uav_command);

        rate.sleep();
        
    }
    return 0;
    

}
