#include <ros/ros.h>
#include <sstream>
#include <Eigen/Eigen>
#include <iostream>
#include <iterator>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/TargetsInFrame.h>
#include <prometheus_msgs/Target.h>

#include <prometheus_msgs/GimbalState.h>
#include <prometheus_msgs/GimbalControl.h>
#include <prometheus_msgs/GimbalFollowTrackResSevice.h>
#include <mission_utils.h>
#include "printf_utils.h"
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include "custom_data_segment_msg.hpp"
#include <yaml-cpp/yaml.h>

#include <prometheus_msgs/TextInfo.h>

#include <prometheus_msgs/Control.h>

#include <std_srvs/SetBool.h>
#include <queue>
#include <algorithm>


#include <vector>
#include <cmath>
using namespace std;
using namespace Eigen;
using namespace cv;
#define NODE_NAME "prosim_gimbal_aruco_landing"
prometheus_msgs::UAVState g_UAVState;
Eigen::Vector3f g_drone_pos;
float height;
prometheus_msgs::UAVControlState g_uavcontrol_state;
prometheus_msgs::Target g_Detection_raw;      //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::TargetsInFrame det;
prometheus_msgs::UAVCommand g_command_now; //发送给控制模块的命令
int g_uav_id;


float gimbal_pitch, gimbal_yaw; // 吊舱PY参数
prometheus_msgs::GimbalState g_GimbalState;
prometheus_msgs::TextInfo text_info;
std_srvs::SetBool gimbal_downlock;
prometheus_msgs::GimbalControl gimbal_control_angle;
prometheus_msgs::GimbalFollowTrackResSevice gimbal_tracking_state;

int tracked_id = -1;






float x_vel = 0.0;
float y_vel = 0.0;
float z_vel = 0.0;
float yaw_rate = 0.0;


// 全局或类成员变量，指数加权平均平滑参数，建议可调


double move_duration = 0; // 在丢失后移动持续时间（秒）
int lost_count_threshold = 0;  // 丢失目标计数阈值（根据频率调节）

float last_tracked_velocity[3] = {0.0, 0.0, 0.0};

int track_lost_count = 0; // 跟踪阶段丢失计数
int land_lost_count = 0;   // 降落阶段丢失计数
bool has_moved_after_lost = false;
ros::Time move_start_time;
bool is_spirecv_cmd = false;
bool Cx_Cy_init = false;


struct Position3D {
    double x;
    double y;
    double z;
};

const size_t MAX_SIZE = 30; // 默认窗口（未使用动态窗口时）
size_t angle_window_size = 30; // 降落趋势角度动态窗口大小
const size_t ANGLE_WINDOW_MIN = 5;
const size_t ANGLE_WINDOW_MAX = 50;

std::vector<Position3D> positions;
double land_angle = 90.0;
// const double verticalThreshold = 15.0;  // 15度以内认为垂直降落





// 计算向量长度
double length(const Position3D& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// 计算两个向量的点积
double dot(const Position3D& a, const Position3D& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 计算降落趋势向量与垂直向量夹角，单位度
double calculateLandingAngle(const Position3D& start, const Position3D& end) {
    Position3D v = {end.x - start.x, end.y - start.y, end.z - start.z};
    double v_len = length(v);
    if (v_len == 0) return 0.0;  // 位置未变化，角度为0

    Position3D vertical = {0, 0, -1};  // 假设垂直向量向下

    double cosTheta = dot(v, vertical) / v_len;  // vertical单位向量长度为1

    if (cosTheta > 1.0) cosTheta = 1.0;
    else if (cosTheta < -1.0) cosTheta = -1.0;

    double angleRad = std::acos(cosTheta);
    double angleDeg = angleRad * 180.0 / M_PI;
    return angleDeg;
}


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
    bool is_update = false;
    for(auto &tar : msg->targets)
    {
        if(!tar.mode && !(tracked_id != -1 && tracked_id == tar.tracked_id)){
                continue;
        }
        g_Detection_raw = tar;
        tracked_id = g_Detection_raw.tracked_id;
        is_update = true;
        break;
    }
    if(!is_update && tracked_id != -1)
    {
        // no-op
    }
}

void GimbalStateCb(const prometheus_msgs::GimbalState::ConstPtr &msg)
{
    g_GimbalState = *msg;
    gimbal_pitch = g_GimbalState.angleRT[1];
    gimbal_yaw = g_GimbalState.angleRT[2];
}

void droneControlStateCb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    g_uavcontrol_state = *msg;
}

inline float clamp(float value, float max)
{
    max = std::abs(max);
    return std::fmin(std::fmax(value,-max),max);
}



// 在全局变量区域添加新的变量
int landing_condition_count = 0;  // 满足降落条件的连续计数
float last_gimbal_pitch = 0.0;  // 上一次的吊舱pitch角

int main(int argc, char **argv)
{
    ros::init(argc,argv,NODE_NAME);
    ros::NodeHandle nh;
        
    float kp_x, kp_gimbal, max_velocity, max_yaw_rate, ignore_error_pitch,
     gimbal_pitch_init,land_cx,land_cy,verticalThreshold,kp_gimbal_single
     ,land_move_duration,land_lost_count_threshold,track_move_duration,track_lost_count_threshold;
    float gimbal_only_pitch_threshold;
    // 降落条件相关参数
    int min_landing_condition_frames;
    float gimbal_pitch_stability_threshold;
    nh.param<float>(ros::this_node::getName() + "/kp_x", kp_x, 100);
    nh.param<float>(ros::this_node::getName() + "/kp_gimbal", kp_gimbal, 0.01);
    nh.param<float>(ros::this_node::getName() + "/max_velocity", max_velocity, 0.5);
    nh.param<float>(ros::this_node::getName() + "/max_yaw_rate", max_yaw_rate, 10);
    nh.param<float>(ros::this_node::getName() + "/ignore_error_pitch", ignore_error_pitch, 10);
    nh.param<float>(ros::this_node::getName() + "/gimbal_pitch_init", gimbal_pitch_init, 20);
    nh.param<float>(ros::this_node::getName() + "/land_cx", land_cx, 0.5);
    nh.param<float>(ros::this_node::getName() + "/land_cy", land_cy, 0.7);
    nh.param<float>(ros::this_node::getName() + "/verticalThreshold", verticalThreshold, 15.0);
    nh.param<float>(ros::this_node::getName() + "/kp_gimbal_single", kp_gimbal_single, 200.0);
    nh.param<float>(ros::this_node::getName() + "/track_lost_count_threshold", land_lost_count_threshold, 60.0);
    nh.param<float>(ros::this_node::getName() + "/track_move_duration", land_move_duration, 2.0);
    nh.param<float>(ros::this_node::getName() + "/land_lost_count_threshold", land_lost_count_threshold, 60.0);
    nh.param<float>(ros::this_node::getName() + "/land_move_duration", land_move_duration, 2.0);
    nh.param<float>(ros::this_node::getName() + "/gimbal_only_pitch_threshold", gimbal_only_pitch_threshold, 40.0);
    // 降落条件相关参数
    nh.param<int>(ros::this_node::getName() + "/min_landing_condition_frames", min_landing_condition_frames, 4);
    nh.param<float>(ros::this_node::getName() + "/gimbal_pitch_stability_threshold", gimbal_pitch_stability_threshold, 6.0);
    nh.param<int>(ros::this_node::getName() + "uav_id", g_uav_id, 1);
    
    // 输出降落条件参数
    printf("[INFO] Landing condition parameters:\n");
    printf("[INFO]   min_landing_condition_frames: %d\n", min_landing_condition_frames);
    printf("[INFO]   gimbal_pitch_stability_threshold: %.1f\n", gimbal_pitch_stability_threshold);
    printf("[INFO]   ignore_error_pitch: %.1f\n", ignore_error_pitch);
    //获取无人机ENU下位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);
    //获取遥控器控制状态
    ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, droneControlStateCb);
    //获取吊舱状态
    ros::Subscriber gimbal_state_sub = nh.subscribe<prometheus_msgs::GimbalState>("/uav" + std::to_string(g_uav_id) + "/gimbal/state", 10, GimbalStateCb);
    //获取视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::TargetsInFrame>("/uav" + std::to_string(g_uav_id) + "/spirecv/target", 10, VisionCb);
    //发布控制指令到uav_controller
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);
    //发布服务让吊舱回中
    ros::ServiceClient gimbal_home_client_ = nh.serviceClient<std_srvs::SetBool>("/uav" + std::to_string(g_uav_id) + "/gimbal_server");
    //发布画面点击
    ros::Publisher target_pub = nh.advertise<prometheus_msgs::Control>("/uav" + std::to_string(g_uav_id) + "/spirecv/control", 1);
    //锁定吊舱角度
    ros::ServiceClient gimbal_downward_lock_client_ = nh.serviceClient<std_srvs::SetBool>("/uav" + std::to_string(g_uav_id) + "/gimbal_downward_lock");
    //反馈给地面站消息
    ros::Publisher text_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(g_uav_id) + "/prometheus/text_info", 1);


    //发布吊舱控制
    ros::Publisher gimbal_pub = nh.advertise<prometheus_msgs::GimbalControl>("/uav" + std::to_string(g_uav_id) + "/gimbal/control", 10);

    ros::ServiceClient gimbal_follow = nh.serviceClient<prometheus_msgs::GimbalFollowTrackResSevice>("/uav" + std::to_string(g_uav_id) + "/gimbal_follow_track_res_server");

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


    
    std::string last_message = "";
    std::string message = "Gimbal Aruco Landing Start!!!";
    uint8_t message_type = prometheus_msgs::TextInfo::INFO;

    // 程序执行状态
    enum State{
        INIT = 0,
        TRACKING = 1,
        LANDING = 2
    };

    State current_state = State::INIT;

    

    int hz = 20;
    ros::Rate rate(hz);
    int i =0;
    bool gimbal_pitch_init_flag = false;
    while(ros::ok())
    {
        ros::spinOnce();

        if(g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL state\n");
        }else
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            g_command_now.Yaw_Rate_Mode = true;
                        
            if(current_state == State::INIT)
            {
                if(std::abs(gimbal_pitch_init -gimbal_pitch ) >  5 && !gimbal_pitch_init_flag)
                {
                    //初始向下度，方便点击目标
                    gimbal_control_angle.header.stamp = ros::Time::now();
                    gimbal_control_angle.mode         = 4;  //angle
                    // gimbal_control_angle.angle[0]     = 0;  //deg
                    gimbal_control_angle.angle[1]     = gimbal_pitch_init;  //deg
                    gimbal_pub.publish(gimbal_control_angle);
                    PCOUT(0, GREEN, "gimbal control !!\n");
                }else{
                    gimbal_pitch_init_flag = true ;
                }


                // 判断是否已经点击目标
                if(g_Detection_raw.mode || g_GimbalState.moveMode == 3)
                {
                    current_state = State::TRACKING;
                }else
                {
                    PCOUT(-1, GREEN, "Waiting for click target && gimbal track\n");
                }
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                g_command_now.Yaw_Rate_Mode = false;

            }else if(current_state == State::TRACKING)
            {
                
                // ... kalman预测和修正逻辑 ...
                // 2.kalman prediction
                Mat prediction = KF.predict();
            
                float measurementX = g_Detection_raw.px;
                float measurementY = g_Detection_raw.py;
                float measurementZ = g_Detection_raw.pz;
                measurement.at<float>(0) = measurementX;
                measurement.at<float>(1) = measurementY;
                measurement.at<float>(2) = measurementZ;
                KF.correct(measurement);
            
                double last_distance = distance;
                distance = sqrt(pow(KF.statePost.at<float>(0), 2) + pow(KF.statePost.at<float>(1), 2) + pow(KF.statePost.at<float>(2), 2));
            
                if (std::abs(distance - last_distance) > 7 )
                {
                    printf("waiting for cov\n");
                    rate.sleep();
                    continue;
                }

                move_duration = track_move_duration; // 在丢失后移动持续时间（秒）
                lost_count_threshold = track_lost_count_threshold;  // 丢失目标计数阈值（根据频率调节）

                // 主循环内逻辑：
                // 失去目标时
                if (g_Detection_raw.mode == false) 
                {
                    track_lost_count++;

                    if (track_lost_count > lost_count_threshold)
                    {
                        g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                        PCOUT(-1, GREEN, "Target lost, hovering...\n");
                        has_moved_after_lost = false;

                    }
                    else
                    {
                        if (!has_moved_after_lost)
                        {
                            move_start_time = ros::Time::now();
                            has_moved_after_lost = true;
                            PCOUT(-1, GREEN, "Target lost long time, start moving...\n");
                        }
                        ros::Duration elapsed = ros::Time::now() - move_start_time;

                        if (g_Detection_raw.mode == true)
                        {
                            track_lost_count = 0;
                            has_moved_after_lost = false;
                            PCOUT(-1, GREEN, "Target recovered during move, resuming tracking...\n");
                            // 这里可以跳出当前循环或直接continue进入目标跟踪逻辑
                        }
                        else if (elapsed.toSec() < move_duration)
                        {
                            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
                            g_command_now.yaw_ref = 0;
                            PCOUT(-1, GREEN, "Moving after lost with last velocity...\n");
                        }
                        else
                        {
                            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                            PCOUT(-1, GREEN, "Moved after lost, target still lost, hovering.\n");
                        }
                    }
                }
                // 识别到目标时
                else 
                {
                    track_lost_count = 0;
                    has_moved_after_lost = false;

                    // 计算当前速度
                    // 控制无人机前进
                    x_vel = kp_x * KF.statePost.at<float>(2);                 
                    
                    if ( (std::abs(90 - gimbal_pitch) < gimbal_only_pitch_threshold))
                    {
                        //固定吊舱航向
                        gimbal_tracking_state.request.action = 2;
                        gimbal_follow.call(gimbal_tracking_state);
                        PCOUT(-1, GREEN, "Gimbal Only Pitch!! \n");
                        y_vel = kp_x * -KF.statePost.at<float>(0);
                        yaw_rate = 0;
                        g_command_now.Yaw_Rate_Mode = false;
                    }else
                    {
                        //不固定吊舱航向
                        gimbal_tracking_state.request.action = 1;
                        gimbal_follow.call(gimbal_tracking_state);
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
                            // su17单轴使用cx体现偏航控制
                            yaw_rate = kp_gimbal_single * (0.5 - g_Detection_raw.cx) ;
                        }
                        g_command_now.Yaw_Rate_Mode = true;                
                    }

                    last_tracked_velocity[0] = clamp(x_vel, max_velocity);
                    last_tracked_velocity[1] = clamp(y_vel, max_velocity);
                    last_tracked_velocity[2] = clamp(z_vel, max_velocity);               
                    PCOUT(-1, GREEN, "Tracking target, moving...\n");                       
                }

                // 判断是否能进入降落状态，只判断吊舱角度
                // 使用平方关系：目标面积越大（距离越近），角度容差越大
                float area = g_Detection_raw.w * g_Detection_raw.h;
                float dynamic_ignore_pitch = std::max(13.0f, area * area * ignore_error_pitch * 200);
                // printf("area * area * ignore_error_pitch * 200 = %f\n", area * area * ignore_error_pitch * 200);

                bool gimbal_condition = std::abs(90 - gimbal_pitch) < dynamic_ignore_pitch;
                
                // 检查吊舱角度稳定性
                bool gimbal_stable = std::abs(gimbal_pitch - last_gimbal_pitch) < gimbal_pitch_stability_threshold;
                
                // 综合判断条件：只判断吊舱角度，不判断水平距离
                bool landing_ready = (gimbal_condition && gimbal_stable);
                
                // 更新降落条件计数
                if (landing_ready) {
                    landing_condition_count++;
                } else {
                    landing_condition_count = 0;
                }
                
                // 保存当前值用于下次比较
                last_gimbal_pitch = gimbal_pitch;
                
                // 最终判断：需要连续满足条件足够长时间
                if(landing_condition_count >= min_landing_condition_frames)
                {
                    // 如果可以进入则先保持悬停
                    g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                    g_command_now.Yaw_Rate_Mode = false;
                    
                    // 输出调试信息
                    printf("[INFO] Landing conditions met: gimbal_pitch=%.1f, count=%d\n", 
                           gimbal_pitch, landing_condition_count);
                    
                    // 取消跟踪进入锁头模式
                    if(g_GimbalState.moveMode == 3)
                    {
                        prometheus_msgs::Control cmd;
                        cmd.mouse = prometheus_msgs::Control::MOUSE_RIGHT;
                        cmd.x = -1;
                        cmd.y = -1;
                        target_pub.publish(cmd);
                        PCOUT(0, GREEN, "cancel tracking!!!\n");
                        message = "cancel tracking!!!";
                        message_type = prometheus_msgs::TextInfo::INFO;
                        is_spirecv_cmd = true;
                    }else{ //进入锁头模式后 进入降落状态
                        if(g_GimbalState.type == 0 || g_GimbalState.type == 3 || g_GimbalState.type == 4)// su17吊舱4可以进入俯拍模式，gx40仿真中3也是这个控制
                        {
                            // 判断角度是否进入俯拍，没有进入则进入
                            if(std::abs(90 - gimbal_pitch) > 2.0)
                            {
                                std_srvs::SetBool lock;
                                lock.request.data = true;
                                gimbal_downward_lock_client_.call(lock);

                                //gimbal_downward_lock
                                PCOUT(0, GREEN, " gimbal downward lock!!!\n");
                                message = " gimbal downward lock!!!";
                                message_type = prometheus_msgs::TextInfo::INFO;
                            }else
                            {
                                // 进入后 进入降落状态
                                current_state = State::LANDING;
                                is_spirecv_cmd = false;
                                // 锁头时确定角度窗口大小（随目标面积设置，使用平方关系）
                                {
                                    double area = (double)g_Detection_raw.w * (double)g_Detection_raw.h;
                                    // 使用平方关系：面积越小（距离越远），窗口越大
                                    double win = (1.0 - area) * (1.0 - area) * 12.0;
                                    win = std::max<double>((double)ANGLE_WINDOW_MIN, std::min<double>((double)ANGLE_WINDOW_MAX, win));
                                    angle_window_size = static_cast<size_t>(win);
                                }
                                positions.clear();

                                PCOUT(0, GREEN, " gimbal lock head!!!\n");
                                message = " gimbal lock head!!!";
                                message_type = prometheus_msgs::TextInfo::INFO;
                            }
                        }else// gx40直接进入降落状态
                            {                                
                                // 进入降落状态
                                current_state = State::LANDING;
                                is_spirecv_cmd = false;
                            }

                        
                    }
                } else {
                    // 输出当前状态信息用于调试
                    if (landing_condition_count > 0) {
                        printf("[WARN] Landing conditions progress: count=%d/%d\n", 
                               landing_condition_count, min_landing_condition_frames);
                        printf("[INFO] gimbal_pitch=%.1f,dynamic_ignore_pitch=%.1f\n", 
                        gimbal_pitch, dynamic_ignore_pitch);
                    }
                }

            }else if(current_state == State::LANDING)
            {
                //更新丢失判断变量
                lost_count_threshold = land_lost_count_threshold;
                move_duration = land_move_duration;
                
                // land目标丢失
                if (det.targets.empty()) 
                {
                    land_lost_count++;

                    if (land_lost_count > lost_count_threshold && land_angle >= verticalThreshold)
                    {
                        g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                        PCOUT(-1, GREEN, "Target lost, Land hovering...\n");
                        has_moved_after_lost = false;
                    }
                    else
                    {
                        if (!has_moved_after_lost)
                        {
                            move_start_time = ros::Time::now();
                            has_moved_after_lost = true;
                        }
                        ros::Duration elapsed = ros::Time::now() - move_start_time;

                        bool in_move_window = (elapsed.toSec() < move_duration) && (land_lost_count <= lost_count_threshold);
                        if (in_move_window)
                        {
                            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
                            g_command_now.yaw_ref = 0;
                            PCOUT(-1, GREEN, "Landing after lost with last velocity...\n");
                        }
                        else
                        {
                            if ((land_angle < verticalThreshold) && has_moved_after_lost)
                            {
                                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                                PCOUT(-1, GREEN, "Find the target and start land!!!\n");
                                message = "Find the target and start land!!!";
                                message_type = prometheus_msgs::TextInfo::INFO;
                            }
                            else
                            {
                                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                                PCOUT(-1, GREEN, "Moved after lost, target still lost, land hovering.\n");
                            }
                        }
                    }
                }
                else 
                {
                    // 识别到目标时,先水平对齐，再开始降落，确保降落趋势正常，防止水平飘
                    land_lost_count = 0;
                    has_moved_after_lost = false;
 
                    // 根据目标ID，查到对应检测的目标的 cx 和 cy 控制无人机 细微调整位置
                    
                    if(std::abs(x_vel) + std::abs(y_vel) <0.1 )
                    {
                        Cx_Cy_init = true;
                    }
                    if(Cx_Cy_init)
                    {
                        //不同大小无人机调整cy的中心,画面上到下为0到1
                        // 使用平方关系计算控制增益，更好地反映距离变化
                        float area = g_Detection_raw.w * g_Detection_raw.h;
                        float distance_factor = (1.0f - area) * (1.0f - area);  // 平方关系
                        
                        x_vel = kp_x * (land_cy - g_Detection_raw.cy) * distance_factor * 1.5f;
                        y_vel = kp_x * (land_cx - g_Detection_raw.cx) * distance_factor * 1.5f;                        
                        
                        // Z速度使用平方关系计算
                        float s_raw = 1.0f - area;      // 目标面积反比因子
                        float s = std::max(0.0f, std::min(1.0f, s_raw));                  // 限制到[0,1]
                        s = s * s;  // 平方关系，增强距离敏感性

                        const float s_thresh = 0.85f;   // 面积阈值(可调)
                        const float z_min   = -0.08f;   // 最小下降速度
                        const float z_mid   = -0.12f;   // 在阈值处的目标速度，保证两段连续
                        const float z_max   = -0.2f;   // 最大下降速度

                        // 分段线性：更细致地映射 s 到 z_vel，保持段间连续
                        float s1 = 0.5f * s_thresh;           // 第一段上界
                        float s2 = s_thresh;                  // 第二段上界（中点）
                        float s3 = 0.5f * (1.0f + s_thresh);  // 第三段上界
                        
                        if (s <= s1) {
                            // 段1：s in [0, s1]，z: z_min → z_min + 0.5*(z_mid - z_min)
                            float t = (s1 > 1e-6f) ? (s / s1) : 1.0f;
                            float z_mid1 = z_min + 0.5f * (z_mid - z_min);
                            z_vel = z_min + t * (z_mid1 - z_min);
                        } else if (s <= s2) {
                            // 段2：s in (s1, s2]，z: z_min + 0.5*(z_mid - z_min) → z_mid
                            float t = (s2 - s1 > 1e-6f) ? ((s - s1) / (s2 - s1)) : 1.0f;
                            float z_mid1 = z_min + 0.5f * (z_mid - z_min);
                            z_vel = z_mid1 + t * (z_mid - z_mid1);
                        } else if (s <= s3) {
                            // 段3：s in (s2, s3]，z: z_mid → (z_mid + z_max)/2
                            float t = (s3 - s2 > 1e-6f) ? ((s - s2) / (s3 - s2)) : 1.0f;
                            float z_mid2 = 0.5f * (z_mid + z_max);
                            z_vel = z_mid + t * (z_mid2 - z_mid);
                        } else {
                            // 段4：s in (s3, 1]，z: (z_mid + z_max)/2 → z_max
                            float t = (1.0f - s3 > 1e-6f) ? ((s - s3) / (1.0f - s3)) : 1.0f;
                            float z_mid2 = 0.5f * (z_mid + z_max);
                            z_vel = z_mid2 + t * (z_max - z_mid2);
                        }
                        // printf("s = %f ,s_thresh = %f  \n",s,s_thresh);

                    }else{
                        // 水平对齐计算速度，使目标在画面中间，避免高度低的时候看不全  
                        // 水平对齐也使用平方关系
                        float area = g_Detection_raw.w * g_Detection_raw.h;
                        float distance_factor = (1.0f - area) * (1.0f - area);  // 平方关系
                        
                        x_vel = kp_x * (0.5 - g_Detection_raw.cy) * distance_factor * 4.0f;
                        y_vel = kp_x * (0.5 - g_Detection_raw.cx) * distance_factor * 4.0f;
                    }
                    last_tracked_velocity[0] = clamp(x_vel, max_velocity);
                    last_tracked_velocity[1] = clamp(y_vel, max_velocity);
                    last_tracked_velocity[2] = clamp(z_vel, max_velocity);
                    PCOUT(-1, GREEN, "Tracking target, landing...\n");                    
                }
                yaw_rate = 0 ;
            }
            //计算降落趋势角度
            // if(Cx_Cy_init && !det.targets.empty())
            if(Cx_Cy_init)
            {
                Position3D p = {g_UAVState.position[0],g_UAVState.position[1], g_UAVState.position[2]};
                // 维护动态窗口大小
                if (positions.size() >= angle_window_size) {
                    positions.erase(positions.begin());
                }
                positions.push_back(p);
                if (positions.size() >= angle_window_size)
                {
                    land_angle = calculateLandingAngle(positions[0], positions[positions.size()-1]);
                    std::cout << "无人机总降落趋势角度: " << land_angle << " 度" << std::endl;
                }
            }
        }
        
        
        g_command_now.velocity_ref[0] = clamp(x_vel, max_velocity);
        g_command_now.velocity_ref[1] = clamp(y_vel, max_velocity);
        g_command_now.velocity_ref[2] = clamp(z_vel, max_velocity);
        g_command_now.yaw_rate_ref = clamp(yaw_rate, max_yaw_rate) * M_PI / 180;
        printf("x_vel_ref = %f [m/s] \n",g_command_now.velocity_ref[0]);
        printf("y_vel_ref = %f [m/s] \n",g_command_now.velocity_ref[1]);
        printf("z_vel_ref = %f [m/s] \n",g_command_now.velocity_ref[2]);
        printf("yaw_rate_ref = %f [ang/s] \n",g_command_now.yaw_rate_ref);
        printf("g_Detection_raw.cx = %f  \n",g_Detection_raw.cx);
        printf("g_Detection_raw.cy = %f  \n",g_Detection_raw.cy);
        printf("angle_window_size = %ld  \n",angle_window_size);

        // Publish
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        command_pub.publish(g_command_now);
        
        // 发布消息反馈
        if(last_message != message && message != "")
        {
            text_info.header.stamp = ros::Time::now();
            text_info.Message = message;
            text_info.MessageType = message_type;
            text_info_pub.publish(text_info);
            last_message = message;
        }

        // 判断是否触发了降落
        bool is_land = g_command_now.Agent_CMD == prometheus_msgs::UAVCommand::Land || g_uavcontrol_state.control_state == prometheus_msgs::UAVControlState::LAND_CONTROL;
        // 控制降落 
        while (is_land)
        {
            // 如果无人机上锁则退出该循环
            if(!g_UAVState.armed)
            {
                // 吊舱是否需要归中
                std_srvs::SetBool set_home;
                set_home.request.data       = true;
                gimbal_home_client_.call(set_home);
                gimbal_tracking_state.request.action = 1;
                gimbal_follow.call(gimbal_tracking_state);
                PCOUT(-1, GREEN, "gimbal_follow.call\n");
                // 初始化
                tracked_id                  = -1;
                current_state               = State::INIT;
                track_lost_count = 0;
                land_lost_count = 0;
                has_moved_after_lost = false;
                g_command_now.Agent_CMD     = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                g_command_now.Yaw_Rate_Mode = false;
                Cx_Cy_init = false;
                // 重置降落条件相关变量
                landing_condition_count = 0;
                last_gimbal_pitch = 0.0;

                // 退出降落循环
                break;
            }
            //command_pub.publish(g_command_now);
            rate.sleep();
            ros::spinOnce();
        }

        rate.sleep();
    }
    return 0;
}

