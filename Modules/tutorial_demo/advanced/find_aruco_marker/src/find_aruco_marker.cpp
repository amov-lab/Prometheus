#include <ros/ros.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/ArucoInfo.h>
#include <sstream>
#include <unistd.h>
#include <math.h>
#include <Eigen/Eigen>
#include "printf_utils.h"
#include <prometheus_msgs/MultiArucoInfo.h>

using namespace std;

const float PI = 3.1415926;

//创建无人机相关数据变量
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;
prometheus_msgs::ArucoInfo now_arucos_info;
// 创建圆形跟踪的相关变量
// 整个圆形的飞行时间
float circular_time;
// 每次控制数据更新时的弧度增量
float angle_increment;
// 无人机的合速度也就是圆的线速度
float line_velocity;
// 无人机的控制频率
float control_rate;
// 圆的半径
float radius;
// 目标二维码id
int goto_id = 1;
// 最大目标丢失计数
constexpr int max_loss_count = 30;
int loss_count = max_loss_count;

//通过设定整个圆的飞行时间,控制频率,圆的半径来获取相关参数
void get_circular_property(float time, int rate, float radius)
{
    //计算角速度(rad/s)
    float w = 2 * PI / time;
    //计算线速度(m/s)
    line_velocity = -radius * w;
    //计算控制数据发布的弧度增量
    angle_increment = w / rate;
}

// 为
void visualFeedbackCallback(const prometheus_msgs::MultiArucoInfo::ConstPtr &msg)
{
    now_arucos_info.detected = false;
    for (auto &aruco : msg->aruco_infos)
    {
        if (aruco.aruco_num != goto_id || !aruco.detected)
            continue;
        now_arucos_info = aruco;
        // 相机坐标到机体坐标系x、y轴需要交换，并且方向相反。
        now_arucos_info.position[0] = -aruco.position[1];
        now_arucos_info.position[1] = -aruco.position[0];
        // <pose>0 0 0.05 0 1.5707963 0</pose>
        // 根据无人机pdf文件，下视觉相机在距离无人机质心0.05米位置
        // 无人机质心到地面距离是 0.09
        now_arucos_info.position[2] = -(aruco.position[2] + 0.05 - 0.09);
        break;
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

//主函数
int main(int argc, char **argv)
{
    // ROS初始化,设定节点名
    ros::init(argc, argv, "find_arcuo_marker");
    //创建句柄
    ros::NodeHandle n;
    // 读取参数

    int uav_id;
    std::string visual_feedback_topic_name;
    n.param("uav_id", uav_id, 1);
    n.param("visual_feedback_topic_name", visual_feedback_topic_name, std::string("/uav1/prometheus/object_detection/arucos_det"));

    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 10);
    //创建无人机状态命令订阅者，使用匿名函数,lambda作为回调函数
    ros::Subscriber uav_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state", 10, [&](const prometheus_msgs::UAVState::ConstPtr &msg)
                                                                           { uav_state = *msg; });
    //创建无人机控制状态命令订阅者，使用匿名函数,lambda作为回调函数
    ros::Subscriber uav_control_state_sub = n.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(uav_id) + "/prometheus/control_state", 10, [&](const prometheus_msgs::UAVControlState::ConstPtr &msg)
                                                                                          { uav_control_state = *msg; });

    // 订阅视觉反馈
    ros::Subscriber visual_feedback_sub = n.subscribe<prometheus_msgs::MultiArucoInfo>(visual_feedback_topic_name, 20, visualFeedbackCallback);

    // 圆轨迹周期
    circular_time = 30;
    control_rate = 60;
    // 圆轨迹半径
    radius = 5;

    // 获取线速度line_velocity， 角速度angle_increment
    get_circular_property(circular_time, control_rate, radius);

    bool circular_success = false;
    int count = 0;
    ros::Rate rate(control_rate);

    Eigen::Vector3d target_pos = {-radius, 0.0, 2.0};

    // 到达那个二维码上方
    if (!checkInput(goto_id))
        return 0;

    enum STATUS
    {
        // 等到到达预设初始点
        WAITE_INIT_POS,
        // 绕圆飞行
        CIRCULAR_SEARCH,
        // 到达指定二维码上方
        TO_DESTINATION,
        // 降落
        LAND,
    };

    STATUS run_status = WAITE_INIT_POS;

    while (ros::ok())
    {
        prometheus_msgs::ArucoInfo now_marker;

        //调用一次回调函数
        ros::spinOnce();
        //检测无人机是否处于[COMMAND_CONTROL]模式
        if (uav_control_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL mode");
            continue;
        }

        std::ostringstream info;

        uav_command.header.stamp = ros::Time::now();
        switch (run_status)
        {
        // 无人机到达预设位置
        case WAITE_INIT_POS:
            //坐标系
            uav_command.header.frame_id = "ENU";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // Move_mode
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            //无人机将会以当前位置移动
            uav_command.position_ref[0] = target_pos[0];
            uav_command.position_ref[1] = target_pos[1];
            uav_command.position_ref[2] = target_pos[2];
            uav_command.yaw_ref = 0.0;

            //打印无人机控制命令发布信息
            PCOUT(1, GREEN, "Go to initial point");

            //检测无人机是否到达初始点
            //当无人机距离目标位置±0.1米范围内时认为到达初始点
            {
                Eigen::Vector3d uav_pos = {uav_state.position[0], uav_state.position[1], uav_state.position[2]};
                float distance = (uav_pos - target_pos).norm();
                if (distance < 0.1)
                {
                    PCOUT(1, GREEN, " UAV arrived at initial point");
                    run_status = CIRCULAR_SEARCH;
                }
            }
            break;
        // 无人机绕圆飞行
        case CIRCULAR_SEARCH:
            //坐标系
            uav_command.header.frame_id = "ENU";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // Move_mode
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
            //无人机按照圆形轨迹飞行

            uav_command.velocity_ref[0] = -line_velocity * std::sin(count * angle_increment);
            uav_command.velocity_ref[1] = line_velocity * std::cos(count * angle_increment);
            uav_command.velocity_ref[2] = 0;
            uav_command.position_ref[2] = target_pos[2];
            //发布的命令ID加1
            //发布降落命令
            ++count;
            info << "Waypoint Tracking > Velocity_x: " << uav_command.velocity_ref[0] << " Veloticy_y: " << uav_command.velocity_ref[1];
            PCOUT(1, GREEN, info.str());
            if (now_arucos_info.detected)
            {
                run_status = TO_DESTINATION;
                loss_count = max_loss_count;
            }
            break;
        // 前往二维码上方
        case TO_DESTINATION:
            if (!now_arucos_info.detected)
            {
                --loss_count;
                if (loss_count < 0)
                    run_status = CIRCULAR_SEARCH;
                break;
            }
            //坐标系
            uav_command.header.frame_id = "BODY";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // 机体系下的速度控制
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY;

            uav_command.velocity_ref[0] = 0.5 * now_arucos_info.position[0];
            uav_command.velocity_ref[1] = 0.5 * now_arucos_info.position[1];
            uav_command.position_ref[2] = target_pos[2];
            info << "Find object,Go to the target point > velocity_x: " << uav_command.velocity_ref[0] << " [m/s] "
                 << "velocity_y: " << uav_command.velocity_ref[1] << " [m/s] "
                 << std::endl;
            PCOUT(1, GREEN, info.str());
            if (std::abs(uav_command.velocity_ref[0]) + std::abs(uav_command.velocity_ref[1]) < 0.04)
                run_status = LAND;
            break;
        case LAND:
            //坐标系
            uav_command.header.frame_id = "BODY";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // 机体系下的速度控制
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            uav_command.velocity_ref[0] = 0.5 * now_arucos_info.position[0];
            uav_command.velocity_ref[1] = 0.5 * now_arucos_info.position[1];
            uav_command.velocity_ref[2] = 0.2 * now_arucos_info.position[2];
            if (uav_state.position[2] < 0.4)
            {
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            }
            PCOUT(-1, GREEN, "LANDING...");
            break;
        }
        uav_command.Command_ID += 1;
        uav_command_pub.publish(uav_command);
        rate.sleep();
    }
    return 0;
}