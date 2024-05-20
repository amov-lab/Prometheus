//相关所需头文件包含

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include "prometheus_msgs/UAVState.h"
#include "prometheus_msgs/UAVCommand.h"
#include "prometheus_msgs/UAVControlState.h"
#include <sstream>
#include <unistd.h>
#include <math.h>
#include <Eigen/Eigen>



//引用spireCV相关头文件，在Cmakeists中加入spirecv_msgs
#include <prometheus_msgs/TargetsInFrame.h>
#include <prometheus_msgs/Target.h>
#include <prometheus_msgs/ROI.h>

#include "printf_utils.h"

#include "mission_utils.h"

using namespace std;
using namespace Eigen;
#define NODE_NAME "circle_crossing"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int g_uav_id;
std_msgs::Bool vision_switch;
float g_camera_offset[3];
prometheus_msgs::UAVCommand g_command_now;
prometheus_msgs::UAVControlState g_uavcontrol_state; // 遥控器状态
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::UAVState g_UAVState; // 无人机状态
Eigen::Matrix3f g_R_Body_to_ENU;      // 无人机机体系至惯性系转换矩阵
//---------------------------------------Vision---------------------------------------------

//定义视觉检测结果及相关参数定义
prometheus_msgs::Target g_Detection_raw; 
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_enu_frame;
Eigen::Vector3f att_enu_frame;
Eigen::Vector3f pos_body_enu_frame;    //原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系
bool is_detected = false;              // 是否检测到目标标志
int num_count_vision_lost = 0;         //视觉丢失计数器
int num_count_vision_regain = 0;       //视觉丢失计数器
float distance_to_setpoint;            //穿圆前与圆圈的距离
float Detection_distance;            //能检测到圆框的最近距离
float circle_distance;            //穿圆飞行距离


//---------------------------------------Track---------------------------------------------
float g_kp_detect[3]; // 控制参数，控制无人机的速度
float g_c_distance[3]; //穿圆判断距离参数
float g_c_velocity[3]; //穿圆速度参数

// 三种状态
enum EXEC_STATE
{
    //等待进入COMMAND_CONTROL
    WAITING,

    //跟踪圆框
    TRACKING,

    //穿越圆框
    ROUND,
};
EXEC_STATE exec_state;


//---------------------------------------Output---------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void EllipseDetectionCb(const prometheus_msgs::TargetsInFrame::ConstPtr &msg)
{
    g_Detection_raw.category = "EllipseDetection";


    for(auto &target : msg->targets)
    {

        g_Detection_raw = target;//以嵌套方式获取msg中数据

    }


    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 相机安装误差 在mission_utils.h中设置，在launch文件中可以直接传参修改
    // Prometheus下控制由于使用mavros通信，而mavros下常使用的是ENU惯性坐标系，以无人机当前位置为原点，机头向前方向为x轴，垂直机头90度向左为y轴，垂直xy平面向上为z轴

    pos_body_frame[0] = g_Detection_raw.pz + g_camera_offset[0];//相机坐标系转化为无人机使用的ENU惯性系，并补偿相机安装偏差
    pos_body_frame[1] = -g_Detection_raw.px + g_camera_offset[1];
    pos_body_frame[2] = -g_Detection_raw.py + g_camera_offset[2];

   
   //根据检测话题获得置信度score分数来判断检测结果的可靠性

    if (g_Detection_raw.score >= 0.9)
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


    //能检测到圆框的最近距离
    nh.param<float>("Detection_distance", Detection_distance, 0.37);

    //穿圆飞行距离
    nh.param<float>("circle_distance", circle_distance, 2.0);

    //追踪控制参数
    nh.param<float>("kpx_detect", g_kp_detect[0], 0.4);
    nh.param<float>("kpy_detect", g_kp_detect[1], 0.8);
    nh.param<float>("kpz_detect", g_kp_detect[2], 0.8);

    //穿圆判断距离参数，单位:m
    nh.param<float>("cx_distance", g_c_distance[0], 0.44);
    nh.param<float>("cy_distance", g_c_distance[1], 0.15);
    nh.param<float>("cz_distance", g_c_distance[2], 0.15);


    //穿圆速度参数，单位:m/s
    nh.param<float>("cx_velocity", g_c_velocity[0], 0.3);
    nh.param<float>("cy_velocity", g_c_velocity[1], 0);
    nh.param<float>("cz_velocity", g_c_velocity[2], 0);


    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置，单位:m
    nh.param<float>("camera_offset_x", g_camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", g_camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", g_camera_offset[2], -0.15);
}

inline void topicSub(ros::NodeHandle &nh)
{
    //【订阅】摄像头视觉获取圆框位置数据

    static ros::Subscriber EllipseDetection_sub = nh.subscribe<prometheus_msgs::TargetsInFrame>("/uav" + std::to_string(g_uav_id) + "/spirecv/ellipse_detection", 10, EllipseDetectionCb);

    //【订阅】无人机状态
    static ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);


    //【订阅】遥控器状态
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
    ros::init(argc, argv, "autonomous_landing");
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

    //将初始状态设置为WAITING，等待进入COMMAND
    exec_state = EXEC_STATE::WAITING;

    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        // 等待进入COMMAND_CONTROL模式
        if (g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            num_count_vision_regain = 0;//避免在进入COMMAND模式前圆框检测结果影响判断，所以每次进入COMMAND，将检测置0，开始检测
            PCOUT(-1, TAIL, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }

        switch (exec_state)
        {
        // 初始状态，等待视觉检测结果
        case WAITING:
        {

            //通过检测圆框结果判断来进入跟踪状态
            if (is_detected)
            {
                exec_state = TRACKING;
                break;
            }
            else 
            {

            //未检测到圆框便初始点悬停等待
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for ellipse detected!");

            }
             break;

        }
        // 追踪状态
        case TRACKING:
        {
            // 当检测到圆框后，正常追踪

            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;//速度模式跟踪圆框
            g_command_now.velocity_ref[0] = g_kp_detect[0] * (pos_body_frame[0] - Detection_distance);//g_kp_detect为速度跟踪比例控制参数，根据实际情况可在launch文件直接调节，Detection_distance为跟踪圆框最近距离，超过距离，摄像头视野范围就在圆内
            g_command_now.velocity_ref[1] = g_kp_detect[1] * pos_body_frame[1];
            g_command_now.velocity_ref[2] = g_kp_detect[2] * pos_body_frame[2];
            g_command_now.yaw_ref = 0;
            PCOUT(-1, GREEN, "ellipse detected!");

            //当检测到圆框位置与无人机位置在最近且趋于居中，可进入穿圆状态
            if (g_Detection_raw.pz <= g_c_distance[0] && g_Detection_raw.px <= g_c_distance[1] && g_Detection_raw.py <= g_c_distance[2])
            {

                distance_to_setpoint =  g_UAVState.position[0];//记录穿圆前无人机自身位置位置
                exec_state = ROUND;
            
            }  
             break;
       
        }

        //穿圆状态
        case ROUND:
        {

            //进入穿圆模式，速度控制无人机笔直向前穿越圆框
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;
            g_command_now.velocity_ref[0] = g_c_velocity[0];//穿圆速度，可以根据实际需求调整
            g_command_now.velocity_ref[1] = g_c_velocity[1];
            g_command_now.velocity_ref[2] = g_c_velocity[2];
            g_command_now.yaw_ref = 0;
            PCOUT(-1, GREEN, "Through the ring!!!");

            //通过穿圆前位置加上期望穿圆距离来对比无人机自身位置，判断是否完成穿圆
            if (g_UAVState.position[0] > distance_to_setpoint + circle_distance)
            {
             g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Land;//当判断完成穿圆后，无人机降落，整个程序结束
             PCOUT(-1, GREEN,  " landing... ");
             break;
                    
            }

        }
            break;
        }

        // 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        g_command_pub.publish(g_command_now);
        rate.sleep();
    }

    return 0;

}
