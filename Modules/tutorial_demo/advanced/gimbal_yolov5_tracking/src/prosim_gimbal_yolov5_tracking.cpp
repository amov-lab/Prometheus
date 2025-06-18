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
#include <prometheus_msgs/GimbalState.h>
#include <mission_utils.h>
#include "printf_utils.h"
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include "custom_data_segment_msg.hpp"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace cv;
#define NODE_NAME "prosim_gimbal_yolov5_tracking"
prometheus_msgs::UAVState g_UAVState;
Eigen::Vector3f g_drone_pos;
float height;
prometheus_msgs::UAVControlState g_uavcontrol_state;
prometheus_msgs::Target g_Detection_raw;      //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::TargetsInFrame det;
prometheus_msgs::UAVCommand g_command_now; //发送给控制模块的命令
int g_uav_id;

float kpx_track, kpy_track, kpz_track; //控制参数 - 比例参数
bool is_detected = false;              // 是否检测到目标标志
float gimbal_roll, gimbal_pitch, gimbal_yaw; //吊舱RYP参数
prometheus_msgs::GimbalState g_GimbalState;

bool is_reload_param = false;
float reload_kp_x, reload_kp_z, reload_kp_gimbal, reload_max_velocity, reload_max_yaw_rate, reload_tracking_distance;

void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    g_UAVState = *msg;
    g_drone_pos[0] = g_UAVState.position[0];
    g_drone_pos[1] = g_UAVState.position[1];
    g_drone_pos[2] = g_UAVState.position[2];
    if(g_UAVState.location_source == prometheus_msgs::UAVState::RTK || g_UAVState.location_source == prometheus_msgs::UAVState::GPS)
    	height = g_UAVState.rel_alt;
    else
    	height = g_UAVState.position[2];
}

void VisionCb(const prometheus_msgs::TargetsInFrame::ConstPtr &msg)
{
    det = *msg;
    // car_id = det.frame_id;
    g_Detection_raw.mode = false;
    for(auto &tar : msg->targets)
    {
        g_Detection_raw = tar;
    }
}

void GimbalStateCb(const prometheus_msgs::GimbalState::ConstPtr &msg)
{
    g_GimbalState = *msg;
    gimbal_roll = g_GimbalState.angleRT[0];
    gimbal_pitch = g_GimbalState.angleRT[1];
    gimbal_yaw = g_GimbalState.angleRT[2];
}

void droneControlStateCb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    g_uavcontrol_state = *msg;
}

void yolov5TrackingParamsCb(const prometheus_msgs::CustomDataSegment::ConstPtr &msg)
{
    // 读取数据
    CustomDataSegmentMSG custom_data_segment(*msg);

    std::string name;
    custom_data_segment.getValue("name",name);
    if(name == "yolov5_tracking_distance")
    {
        // 分为两种模式
        // 一种为我们默认提供的两种跟踪距离的参数(5m和20m)，另外一种为可控的根据用户的实际需求可动态调整的根据距离参数
        std::string mode;
        custom_data_segment.getValue("mode",mode);
        // 默认跟踪参数
        if(mode == "default_tracking_param")
        {
            // 跟踪距离参数 然后判断吊舱的类型(G1,GX40) 读取对应的yaml文件
            std::string url;
            if(custom_data_segment.getValue("url",url))
            {
                if(g_GimbalState.type == 0)//g1
                    url = "/home/amov/p600_experiment/src/p600_experiment/config/mission_config/g1_" + url;
                else if(g_GimbalState.type == 3)//gx40
                    url = "/home/amov/p600_experiment/src/p600_experiment/config/mission_config/gx40_" + url;
            
            
            	//std::cout << "url = " << url.c_str() << std::endl;
                // 判断url路径的文件是否存在
                try{
                    YAML::Node yamlConfig = YAML::LoadFile(url);
                    reload_kp_x = yamlConfig["kp_x"].as<float>();
                    reload_kp_z = yamlConfig["kp_z"].as<float>();
                    reload_kp_gimbal = yamlConfig["kp_gimbal"].as<float>();
                    reload_max_velocity = yamlConfig["max_velocity"].as<float>();
                    reload_max_yaw_rate = yamlConfig["max_yaw_rate"].as<float>();
                    reload_tracking_distance = yamlConfig["tracking_distance"].as<float>();
                    
                    //std::cout << "reload_tracking_distance = " << reload_tracking_distance <<  std::endl;
                    is_reload_param = true;
                    return;
                }catch(YAML::TypedBadConversion<string> &e){
                    std::cout << "param load faild!!!" << std::endl;
                }
            }
        }
        // 自定义跟踪参数
        else if(mode == "custom_tracking_param")
        {
            // 如果参数都存在
            if(custom_data_segment.getValue("kp_x",reload_kp_x) && custom_data_segment.getValue("kp_z",reload_kp_z)
                && custom_data_segment.getValue("kp_gimbal",reload_kp_gimbal) && custom_data_segment.getValue("max_velocity",reload_max_velocity)
                && custom_data_segment.getValue("max_yaw_rate",reload_max_yaw_rate) && custom_data_segment.getValue("tracking_distance",reload_tracking_distance)
            ){
                // 可以重新加载参数
                is_reload_param = true;
                return;
            }
        }
    }
    is_reload_param = false;
}

inline float clamp(float value, float max)
{
    max = std::abs(max);
    return std::fmin(std::fmax(value,-max),max);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"prosim_gimbal_yolov5_tracking");
    ros::NodeHandle nh;
    
    float kp_x, kp_z, kp_gimbal, max_velocity, max_yaw_rate, expect_height, tracking_distance;
    nh.param<float>(ros::this_node::getName() + "/kp_x", kp_x, 100);
    nh.param<float>(ros::this_node::getName() + "/kp_z", kp_z, 0.005);
    nh.param<float>(ros::this_node::getName() + "/kp_gimbal", kp_gimbal, 0.01);
    nh.param<float>(ros::this_node::getName() + "/max_velocity", max_velocity, 1);
    nh.param<float>(ros::this_node::getName() + "/max_yaw_rate", max_yaw_rate, 10);
    nh.param<float>(ros::this_node::getName() + "/tracking_distance", tracking_distance, 5);
    nh.param<int>(ros::this_node::getName() + "uav_id", g_uav_id, 1);
    //获取无人机ENU下位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);
    //获取遥控器控制状态
    ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, droneControlStateCb);
    //获取吊舱状态
    ros::Subscriber gimbal_state_sub = nh.subscribe<prometheus_msgs::GimbalState>("/uav" + std::to_string(g_uav_id) + "/prosim/gimbal/state", 10, GimbalStateCb);
    //获取视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::TargetsInFrame>("/uav" + std::to_string(g_uav_id) + "/spirecv/target", 10, VisionCb);
    //发布控制指令到uav_controller
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);
    //订阅地面站修改跟踪距离等消息参数
    ros::Subscriber yolov5_tracking_param_sub = nh.subscribe<prometheus_msgs::CustomDataSegment>("/uav" + std::to_string(g_uav_id) + "/prometheus/customdatasegment", 10, yolov5TrackingParamsCb);
    //发布控制指令到uav_controller

    //1.kalman filter setup
    const int stateNum=6;                                      //状态值6×1向量(x,y,z,△x,△y,△z)
    const int measureNum=3;                                    //测量值3×1向量(x,y,z)	
    KalmanFilter KF(stateNum, measureNum, 0);	
    KF.transitionMatrix = (cv::Mat_<float>(6, 6) <<  
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
    Mat statenum = Mat::zeros(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义   

    double distance = 0;// 估计距离
    float gimbal_height; //吊舱高度
    //height = 1;

    ros::Rate rate(50);
    while(ros::ok())
    {
        ros::spinOnce();

        // 重新加载参数
        if(is_reload_param)
        {
            kp_x = reload_kp_x;
            kp_z = reload_kp_z;
            kp_gimbal = reload_kp_gimbal;
            max_velocity = reload_max_velocity;
            max_yaw_rate = reload_max_yaw_rate;
            tracking_distance = reload_tracking_distance;
            is_reload_param = false;
        }
        gimbal_height=height-0.2;
        
        //std::cout << "kp_x = " << kp_x << std::endl;
        //std::cout << "kp_z = " << kp_z << std::endl;
        //std::cout << "kp_gimbal = " << kp_gimbal << std::endl;
        //std::cout << "max_velocity = " << max_velocity << std::endl;
        //std::cout << "max_yaw_rate = " << max_yaw_rate << std::endl;
        //std::cout << "tracking_distance = " << tracking_distance << std::endl;
        
        if(g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL state");
            // expect_height = height;
            expect_height = 4.5;
        }
        if(!g_Detection_raw.mode || g_GimbalState.moveMode != 3)
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for click target && gimbal track");
        }else if(gimbal_pitch <= 0.0 || gimbal_height < 0.0)
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for gimbal_pitch > 0 && height > 0.0");
        }
        else
        {
            //2.kalman prediction
            Mat prediction = KF.predict();
            Point3f predict_pt = Point3f(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2) );   //预测值(x',y',z')
            //3.update Position
            printf("pitch=%f \n",gimbal_pitch);
            printf("yaw=%f \n",gimbal_yaw);
            printf("tracking_distance=%f \n",tracking_distance);
            
            
            float measurementX = gimbal_height * tan((90-gimbal_pitch)*M_PI / 180.0) * sin(gimbal_yaw*M_PI / 180.0);
            float measurementY = gimbal_height;
            float measurementZ = gimbal_height * tan((90-gimbal_pitch)*M_PI / 180.0) * cos(gimbal_yaw*M_PI / 180.0);
            measurement.at<float>(0) = measurementX;
            measurement.at<float>(1) = measurementY;
            measurement.at<float>(2) = measurementZ;
            //4.update
            KF.correct(measurement);
            double last_distance = distance;
            distance = sqrt(KF.statePost.at<float>(0)*KF.statePost.at<float>(0)+KF.statePost.at<float>(1)*KF.statePost.at<float>(1)+KF.statePost.at<float>(2)*KF.statePost.at<float>(2));
              
            // 如果前后两帧差距过大咋认为其还在数据收敛阶段
            if(std::abs(distance - last_distance) > 5)
            {
                rate.sleep();
            	continue;
            }

              
            printf("kalman Position = (x, y, z) = (%.3f, %.3f, %.3f)\n",KF.statePost.at<float>(0),KF.statePost.at<float>(1),KF.statePost.at<float>(2));   //修正后值(x'',y'',z'')
            printf("distance=%f\n",distance);
            printf("g_Detection_raw.pz=%f\n",g_Detection_raw.pz);
            
        
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            g_command_now.Yaw_Rate_Mode = true;
            float x_vel = 0;
            //  = kp_x * (g_Detection_raw.pz - 5.0);
            float y_vel = 0;
            float z_vel = kp_z * (expect_height - g_drone_pos[2]);
            float yaw_rate;
            std::string ss;
            //在跟踪距离5米范围内如果识别结果在±2以内，不控制无人机前进速度，解决无人机x速度控制与yaw控制实时抖动问题
            if(distance > (tracking_distance - 2) && distance < (tracking_distance + 2))
            {
            	if(gimbal_yaw > -5 && gimbal_yaw < 5)
            	{
            	    //x_vel = kp_x * (g_Detection_raw.pz - 5.0);
            	    x_vel = kp_x * (distance - tracking_distance);
            	}
                else
                   x_vel = 0;
            }else{
                //x_vel = kp_x * (g_Detection_raw.pz - 5.0);
                x_vel = kp_x * (distance - tracking_distance);
            }
            // 吊舱的yaw控制SDK左右相反
            if(g_GimbalState.type == 0)
            {
                // G1 yaw control
                yaw_rate = -kp_gimbal * gimbal_yaw;
            }
            else if(g_GimbalState.type == 3)
            {
                // GX40 yaw control
                yaw_rate = kp_gimbal * gimbal_yaw;
            }else
            {
                PCOUT(-1, GREEN, "gimbal type error, cann't control gimbal yaw!");
            }
            g_command_now.velocity_ref[0] = clamp(x_vel, max_velocity);
            g_command_now.velocity_ref[1] = clamp(y_vel, max_velocity);
            g_command_now.velocity_ref[2] = clamp(z_vel,0.2);
            g_command_now.yaw_rate_ref = clamp(yaw_rate, max_yaw_rate) * M_PI / 180;

            
            //ss << "target distance estimate: " << expect_height * std::tan(gimbal_pitch * M_PI / 180.) << "[m]\n"
               //<< "spirecv detection estimate: " << g_Detection_raw.pz << "[m]\n";
            PCOUT(-1, GREEN, expect_height * std::tan(gimbal_pitch * M_PI / 180.));
            //PCOUT(-1, GREEN, "target tracking!");
        }
        // Publish
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        if (g_command_now.Command_ID < 10)
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        command_pub.publish(g_command_now);
        rate.sleep();
    }
    return 0;
}
