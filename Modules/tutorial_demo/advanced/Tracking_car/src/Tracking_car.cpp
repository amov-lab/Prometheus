//相关所需头文件包含

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include "prometheus_msgs/UAVState.h"
#include "prometheus_msgs/UAVCommand.h"
#include "prometheus_msgs/UAVControlState.h"



//引用spireCV相关头文件，在Cmakeists中加入spirecv_msgs
#include <prometheus_msgs/TargetsInFrame.h>
#include <prometheus_msgs/Target.h>
#include <prometheus_msgs/ROI.h>

#include "printf_utils.h"

#include "mission_utils.h"

using namespace std;
using namespace Eigen;
#define NODE_NAME "Tracking_car"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int g_uav_id;
std_msgs::Bool vision_switch;
float g_camera_offset[3];
prometheus_msgs::UAVCommand g_command_now;
prometheus_msgs::UAVControlState g_uavcontrol_state; // 遥控器状态
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::UAVState g_UAVState; // 无人机状态
Eigen::Matrix3f g_R_Body_to_ENU;      // 无人机机体系至惯性系转换矩阵


//定义视觉检测结果及相关参数定义
prometheus_msgs::Target g_Detection_raw; 
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_enu_frame;
Eigen::Vector3f att_enu_frame;
Eigen::Vector3f pos_body_enu_frame;    //原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系
bool is_detected = false;              // 是否检测到目标标志
int num_count_vision_lost = 0;         //视觉丢失计数器
int num_count_vision_regain = 0;       //视觉丢失计数器



//---------------------------------------Track---------------------------------------------
float g_kp_track[3]; // 控制参数，控制无人机的速度
float keep_distance[3];            //跟踪距离保持

// 三种状态
enum EXEC_STATE
{
     //等待进入COMMAND_CONTROL
    WAITING,

    //点击跟踪
    TRACKING,

    //丢失处理
    LOST,
};
EXEC_STATE exec_state;


//---------------------------------------Output---------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TrackCarCb(const prometheus_msgs::TargetsInFrame::ConstPtr &msg)
{
    g_Detection_raw.category = "TrackCar";


    for(auto &target : msg->targets)
    {
 
        g_Detection_raw = target;//以嵌套方式获取msg中数据

    }


    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // Prometheus下控制由于使用mavros通信，而mavros下常使用的是ENU惯性坐标系，以无人机当前位置为原点，机头向前方向为x轴，垂直机头90度向左为y轴，垂直xy平面向上为z轴
    // 相机安装误差 在mission_utils.h中设置场
    // 注意这个是向下摄像头

    pos_body_frame[0] = -g_Detection_raw.py + g_camera_offset[0];
    pos_body_frame[1] = -g_Detection_raw.px + g_camera_offset[1];
    pos_body_frame[2] = -g_Detection_raw.pz + g_camera_offset[2];


    //根据检测话题获得置信度score分数来判断检测结果的可靠性
    if (g_Detection_raw.score == 0)
    {
        num_count_vision_regain++;
        num_count_vision_lost = 0;
    }
    else
    {
        num_count_vision_regain = 0;
        num_count_vision_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if (num_count_vision_lost > VISION_THRES)
    {
        is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if (num_count_vision_regain > VISION_THRES)
    {
        is_detected = true;
    }
}

//获取无人机自身状态数据
void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    g_UAVState = *msg;

    g_R_Body_to_ENU = get_rotation_matrix(g_UAVState.attitude[0], g_UAVState.attitude[1], g_UAVState.attitude[2]);
}

inline void readParams(const ros::NodeHandle &nh)
{
    nh.param<int>("uav_id", g_uav_id, 1);

    //追踪控制参数
    nh.param<float>("kpx_track", g_kp_track[0], 0.6);
    nh.param<float>("kpy_track", g_kp_track[1], 0.6);
    nh.param<float>("kpz_track", g_kp_track[2], 0.3);


    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param<float>("camera_offset_x", g_camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", g_camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", g_camera_offset[2], 0.0);

    //跟踪距离保持
    nh.param<float>("keep_x_distance", keep_distance[0], 0.0);
    nh.param<float>("keep_y_distance", keep_distance[1], 0.0);
    nh.param<float>("keep_z_distance", keep_distance[2], 1.5);
}

inline void topicSub(ros::NodeHandle &nh)
{
    
    //【订阅】摄像头视觉获取小车位置数据
    static ros::Subscriber TrackCar_sub = nh.subscribe<prometheus_msgs::TargetsInFrame>("/uav" + std::to_string(g_uav_id) + "/spirecv/car_detection_with_tracking", 10, TrackCarCb);

    // 无人机状态
    static ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);
                                           

    // 订阅遥控器状态
    static ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, [&](const prometheus_msgs::UAVControlState::ConstPtr &msg) -> void
                                                                                                  { g_uavcontrol_state = *msg; });
}

static ros::Publisher g_vision_switch_pub, g_command_pub;

inline void topicAdv(ros::NodeHandle &nh)
{

    //【发布】发送给控制模块命令
    g_command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "TrackCar");
    ros::NodeHandle nh("~");

    // 节点运行频率： 30hz
    ros::Rate rate(30.0);

    // 读取配置参数
    readParams(nh);
    // 订阅话题
    topicSub(nh);
    // 发布话题
    topicAdv(nh);

    g_command_now.Command_ID = 1;
    exec_state = EXEC_STATE::WAITING;
    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        // 等待进入COMMAND_CONTROL模式
        if (g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, TAIL, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }

        switch (exec_state)
        {
        // 初始状态，等待视觉检测结果
        case WAITING:
        {

            //通过点击检测结果判断来进入跟踪状态
            if(g_Detection_raw.mode == true && is_detected == true)
            {

                exec_state = TRACKING;

                break;
            }
     
            // 默认高度为2米，Modules/uav_control/launch/uav_control_outdoor.yaml
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for car detected!");
             break;
        }
        // 追踪状态
        case TRACKING:
        {

            // 丢失,进入LOST状态
            if (g_Detection_raw.mode == false || is_detected == false)
            {
                exec_state = LOST;
                PCOUT(0, YELLOW, "Lost the Car.");
                break;
                
            }

            // 正常追踪

            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;//速度模式跟踪
            g_command_now.velocity_ref[0] = g_kp_track[0] * pos_body_frame[0];//g_kp_track为速度跟踪比例控制参数，根据实际情况可在launch文件直接调节
            g_command_now.velocity_ref[1] = g_kp_track[1] * pos_body_frame[1];
            g_command_now.velocity_ref[2] = g_kp_track[2] * (pos_body_frame[2] + keep_distance[2]);
            g_command_now.yaw_ref = 0;
            PCOUT(-1, GREEN, "Car detected!");

            break;
       
        }


        // 目标丢失常识自动找回，在丢失目标后无人机先原定悬停一段时间，如果悬停到一段时候后
        // 仍然没有找到目标，则无人机回到初始原点，等待用户操作

        case LOST:
        {
            static int lost_time = 0;
            lost_time++;

            // 首先是悬停等待 尝试得到图像, 如果仍然获得不到图像 则回到原点
            if (lost_time < 20.0)
            {
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;

                ros::Duration(0.4).sleep();

                PCOUT(0, YELLOW, "Try reclicking the car");
            }
            else
            {

                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;

                ros::Duration(0.4).sleep();

                PCOUT(0, YELLOW, "Reselection car");
            }

            // 重新获得信息,进入TRACKING
            if (g_Detection_raw.mode == true)
            {
                exec_state = TRACKING;
                PCOUT(0, GREEN, "Regain the car.");
            }
            break;
        }

        }

        // 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        g_command_pub.publish(g_command_now);
        rate.sleep();
    }

    return 0;

}
