/******************************************************************************
*例程简介: 根据spirecv-ros检测节点发布的目标位置信息，控制机体对目标进行跟踪
*
*效果说明: 机体将根据目标相对距离，计算各个方向的运行速度，以实现对目标的跟踪。
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/
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
#define NODE_NAME "prosim_yolov5_tracking"
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
int num_count_vision_lost = 0;         // 视觉丢失计数器
int num_count_vision_regain = 0;       // 视觉丢失计数器
int Thres_vision = 0; // 视觉丢失计数器阈值

int uav_id;// 无人机id
float distance_to_setpoint; // 相对原点距离
prometheus_msgs::TargetsInFrame det; // 单张图像的检测信息

// 无人机状态回调函数
void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    drone_pos[0] = uav_state.position[0];
    drone_pos[1] = uav_state.position[1];
    drone_pos[2] = uav_state.position[2];
}
// spirecv-ros检测结果回调函数
void VisionCb(const prometheus_msgs::TargetsInFrame::ConstPtr &msg)
{
    det = *msg;
    det_res.mode = false;
    // 跟踪模式仅含有单个目标的信息
    for(auto &tar : msg->targets)
    {
        det_res = tar;
    }
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
    ros::Rate rate(10);


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
    float max_velocity = 5;

    // 获取无人机ENU下的位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state", 10, droneStateCb);
    // 获取spirecv-ros视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/prosim_detection_tracking_with_d435i", 10, VisionCb);
    // 发送给prometheus_uav_control的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 10);
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


    while (ros::ok())
    {   
        //调用一次回调函数
        ros::spinOnce();
        //检测无人机是否处于[COMMAND_CONTROL]模式
        if(uav_control_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL state");
        }
        // 判断spirecv-ros中检测节点是否接收到了鼠标点击
        if(!det_res.mode)
        {
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for click target!");
        }
        else
        {

            // 2.kalman prediction
            Mat prediction = KF.predict();
            // Point3f predict_pt = Point3f(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2) );   //预测值(x',y',z')
            // 3.update Position

            float measurementX          = det_res.px;
            float measurementY          = det_res.py;
            float measurementZ          = det_res.pz;
            measurement.at<float>(0)    = measurementX;
            measurement.at<float>(1)    = measurementY;
            measurement.at<float>(2)    = measurementZ;
            //4.status update
            KF.correct(measurement);
            double last_distance        = distance;
            distance                    = sqrt(KF.statePost.at<float>(0)*KF.statePost.at<float>(0)+KF.statePost.at<float>(1)*KF.statePost.at<float>(1)+KF.statePost.at<float>(2)*KF.statePost.at<float>(2));
 
            // 如果前后两帧差距过大则认为其还在数据收敛阶段
            if(std::abs(distance - last_distance) > 7)
            {
                printf("waiting for cov\n");
                rate.sleep();
            	continue;
            }
            
            printf("ori px, py, pz: %.2f, %.2f, %.2f\n", det_res.px, det_res.py, det_res.pz);
            float px = prediction.at<float>(0);
            float py = prediction.at<float>(1);
            float pz = prediction.at<float>(2);
            printf("pred px, py, pz: %.2f, %.2f, %.2f\n", px, py, pz);

            // 注意区分相机系、机体系、惯性系
            // 相机系，x:右方为正，y：下方为正，z：前方为正
            // 机体系，x:前方为正，y：左方为正，z：上方为正
            // 惯性系（ENU），x:东方为正，y：北方为正，z：上方为正


            // 根据设定跟踪距离计算应该给于无人机的速度
            // 速度vel是机体系坐标,det_res为相机系下目标相对相机的坐标, 
            float x_vel                 = 0;
            float y_vel                 = 0;
            float z_vel                 = 0;

            // // 机体系x对应相机系z
            // if(fabs(tracking_delta[2]-pz)<1 ){
            //     x_vel = kpz_track*  pow((pz-tracking_delta[2]),2)*fabs(pz-tracking_delta[2])/(pz-tracking_delta[2])  ;
            // }else{
            //     x_vel = kpz_track * (pz-tracking_delta[2]);
            // }
            // // 机体系y对应相机系-x
            // if(fabs(tracking_delta[0]-px)<1 ){
            //     y_vel = -kpx_track * pow((px-tracking_delta[0]),2)*fabs(px-tracking_delta[0])/(px-tracking_delta[0]) ;
            // }else{
            //     y_vel = -kpx_track * (px-tracking_delta[0]);
            //}

            // 机体系x对应相机系z
            if(fabs(tracking_delta[2]-pz)<2 ){
                x_vel = kpz_track*  (2-sqrt((4-pow((pz-tracking_delta[2]),2))))*fabs(pz-tracking_delta[2])/(pz-tracking_delta[2])  ;
            }else{
                x_vel = kpz_track * (pz-tracking_delta[2]);
            }
            // 机体系y对应相机系-x
            if(fabs(tracking_delta[0]-px)<2 ){
                y_vel = -kpx_track * (2-sqrt((4-pow((px-tracking_delta[0]),2))))*fabs(px-tracking_delta[0])/(px-tracking_delta[0]) ;
            }else{
                y_vel = -kpx_track * (px-tracking_delta[0]);
            }

            // 机体系z对应相机系-y
            if(fabs(tracking_delta[1]-uav_state.position[2])<0.3){
                z_vel = 0;
            }else{
                z_vel = -kpy_track*(uav_state.position[2]-tracking_delta[1])/3;
            }

            // 设置移动命令
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // 设置移动方式
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            uav_command.yaw_ref = 0;
            // 设置移动速度, velocity_ref 为机体系的速度
            uav_command.velocity_ref[0] = clamp(x_vel, max_velocity);
            uav_command.velocity_ref[1] = clamp(y_vel, max_velocity);
            uav_command.velocity_ref[2] = clamp(z_vel, max_velocity);

            printf("current height %.2f \n", uav_state.position[2]);
            printf("Tracking  los_ay: %.2f, vf_0, vf_1, vf_2: %.2f, %.2f, %.2f ", det_res.los_ay,uav_command.velocity_ref[0], uav_command.velocity_ref[1], uav_command.velocity_ref[2]);
            printf("px, py, pz: %.2f, %.2f, %.2f\n", px, py, pz);

        
        }
        
        // 时间戳
        uav_command.header.stamp = ros::Time::now();
        // 发布的命令ID加1
        uav_command.Command_ID = uav_command.Command_ID + 1;
        // 认为开始机体状态可能不稳定，等待一段时间后进行跟踪。
        if (uav_command.Command_ID < 5)
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        // 发布移动命令
        command_pub.publish(uav_command);


        rate.sleep();
        
    }
    return 0;
    

}
