/******************************************************************************
*例程简介: 第二届大学生未来飞行器挑战赛的实践类仿真demo
*
*效果说明: 无人机由指定的位置一键起飞后，立即转换为自动模式，开始通过机载传感器
*         自主搜索这些目标标靶，并向目标标靶投掷模拟子弹；完成所有的投掷任务后，
*         自主回到起飞点降落。
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/
#include <ros/ros.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/MultiDetectionInfo.h>
#include <sstream>
#include <unistd.h>
#include <math.h>
#include <Eigen/Eigen>
#include "printf_utils.h"
#include "mission_utils.h"

using namespace std;

#define VISION_THRES 10
#define HOLD_THRES 60
const float PI = 3.1415926;

//创建无人机相关数据变量
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;
Eigen::Matrix3f R_Body_to_ENU;      // 无人机机体系至惯性系转换矩阵
prometheus_msgs::DetectionInfo ellipse_det;
float camera_offset[3];
//static target1
// 最大目标丢失计数
constexpr int max_loss_count = 30;
int loss_count = max_loss_count;
int num_lost = 0;          //视觉丢失计数器
int num_regain = 0; 
bool is_detected = false;
int hold_count = 0;
int hold_lost = 0;
bool is_holded = false;


void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    R_Body_to_ENU = get_rotation_matrix(uav_state.attitude[0],uav_state.attitude[1],uav_state.attitude[2]);

}

//无人机控制状态回调函数
void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

void ellipse_det_cb(const prometheus_msgs::MultiDetectionInfo::ConstPtr &msg)
{
    // g_ellipse_det. = false;
    for(auto &ellipes : msg->detection_infos)
    {
        ellipse_det = ellipes;
        if (ellipse_det.detected && ellipse_det.object_name == "T")
        {
            num_regain++;
            num_lost = 0;
        }
        else{
            num_regain = 0;
            num_lost++;
        }
        if(num_lost > VISION_THRES)
        {
            is_detected = false;
            // PCOUT(1, GREEN, "no detect");
        }
        if(num_regain > VISION_THRES){
            is_detected = true;
            // PCOUT(1, GREEN, "detected");
        }
        ellipse_det.sight_angle[0] = ellipes.sight_angle[1];
        ellipse_det.sight_angle[1] = ellipes.sight_angle[0];
        // ellipse_det.position[2] = -ellipes.position[2];
    }
    
}
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
//通过设定整个圆的飞行时间,控制频率,圆的半径来获取相关参数
void get_circular_property(float time, int rate, float radius)
{
    //计算角速度(rad/s)
    float w = 2 * PI / time;
    //计算线速度(m/s)
    line_velocity = radius * w;
    //计算控制数据发布的弧度增量
    angle_increment = w / rate;
}

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "future_aircraft");
    //创建句柄
    ros::NodeHandle n;
    //声明起飞高度,无人机id变量
    float takeoff_height;
    int uav_id;
    //获取起飞高度参数
    ros::param::get("/uav_control_main_1/control/Takeoff_height", takeoff_height);
    ros::param::get("~uav_id", uav_id);
    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    n.param<float>("camera_offset_x", camera_offset[0], 0.0);
    n.param<float>("camera_offset_y", camera_offset[1], 0.0);
    n.param<float>("camera_offset_z", camera_offset[2], 0.0);
    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav1/prometheus/command", 10);
    //创建无人机状态命令订阅者
    ros::Subscriber uav_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav1/prometheus/state", 10, uav_state_cb);
    //创建无人机控制状态命令订阅者
    ros::Subscriber uav_control_state_sub = n.subscribe<prometheus_msgs::UAVControlState>("/uav1/prometheus/control_state", 10, uav_control_state_cb);

    ros::Subscriber ellipse_det_sub = n.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/ellipse_det", 10, ellipse_det_cb);
    
    // 圆轨迹周期
    circular_time = 40;
    control_rate = 20;
    // 圆轨迹半径
    radius = 2.5;

    // 获取线速度line_velocity， 角速度angle_increment
    get_circular_property(circular_time, control_rate, radius);
    int count = 0;
    bool circular_success = false;
    
    //循环频率设置为**HZ
    ros::Rate r(control_rate);

    Eigen::Vector3d waypoint1 = {-4.5, -3.0, 1.5};
    Eigen::Vector3d waypoint2 = {-4.5, 3.0, 1.5};
    Eigen::Vector3d waypoint3 = {4.5, 0, 2.3};
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    //打印demo相关信息
    cout << GREEN << " [fly race demo " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Level: [Advance] " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Please use the RC SWA to armed, and the SWB to switch the drone to [COMMAND_CONTROL] mode  " << TAIL << endl;

    // 四种状态机
    enum EXEC_STATE
    {
        TAKEOFF,
        WAY1,
        WAY2,
        WAY3,
        SEARCH,
        SEARCH_MOVING,
        TRACKING,
        TRACKING_MOVING,
        LOST,
        RETURN,
    };
    EXEC_STATE exec_state = TAKEOFF;//志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标

    while(ros::ok())
    {
        //调用一次回调函数
        ros::spinOnce();
        // 等待进入COMMAND_CONTROL模式
        if (uav_control_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, TAIL, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }
        // else{
        //     uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        //     PCOUT(-1, TAIL, "hold for now");
        // }

        std::ostringstream info;
        uav_command.header.stamp = ros::Time::now();
        switch (exec_state)
        {
        case TAKEOFF:
            uav_command.header.frame_id = "ENU";
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            PCOUT(1, GREEN, "Go to takeoff point");
            // info << "height is : " << uav_state.position[2];
            // PCOUT(1, GREEN, info.str());
            {
                if(fabs(uav_state.position[2] - takeoff_height) < 0.1)
                {
                    PCOUT(1, GREEN, " UAV arrived at takeoff point");
                    exec_state = WAY1;
                }
            }
            break;
        case WAY1:
            uav_command.header.frame_id = "ENU";
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = waypoint1[0];
            uav_command.position_ref[1] = waypoint1[1];
            uav_command.position_ref[2] = waypoint1[2];
            uav_command.yaw_ref = 0.0;
            PCOUT(1, GREEN, "Waypoint1 ");
            // exec_state = SEARCH;
            {
                Eigen::Vector3d uav_pos = {uav_state.position[0], uav_state.position[1], uav_state.position[2]};
                float distance = (uav_pos - waypoint1).norm();
                if (distance < 0.1)
                {
                    // sleep(10);
                    PCOUT(1, GREEN, " UAV arrived at waypoint1 point");
                    exec_state = SEARCH;
                    count = 0;
                }
            }
            break;
        case WAY2:
            uav_command.header.frame_id = "ENU";
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = waypoint2[0];
            uav_command.position_ref[1] = waypoint2[1];
            uav_command.position_ref[2] = waypoint2[2];
            uav_command.yaw_ref = 0.0;
            PCOUT(1, GREEN, "Waypoint2 ");
            {
                Eigen::Vector3d uav_pos = {uav_state.position[0], uav_state.position[1], uav_state.position[2]};
                float distance = (uav_pos - waypoint2).norm();
                if (distance < 0.2)
                {
                    sleep(10);
                    PCOUT(1, GREEN, " UAV arrived at waypoint2 point");
                    exec_state = SEARCH;
                    count = 0;
                }
            }
            break;
        case WAY3:
            uav_command.header.frame_id = "ENU";
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = waypoint3[0];
            uav_command.position_ref[1] = waypoint3[1];
            uav_command.position_ref[2] = waypoint3[2];
            uav_command.yaw_ref = 0.0;
            PCOUT(1, GREEN, "Waypoint3 ");
            {
                Eigen::Vector3d uav_pos = {uav_state.position[0], uav_state.position[1], uav_state.position[2]};
                float distance = (uav_pos - waypoint3).norm();
                if (distance < 0.2)
                {
                    sleep(10);
                    PCOUT(1, GREEN, " UAV arrived at waypoint3 point");
                    exec_state = SEARCH_MOVING;
                }
            }
            break;
        case SEARCH:
            // sleep(10);
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
            uav_command.position_ref[2] = 1.5;
            //发布的命令ID加1
            //发布降落命令
            //计数器
            if(count > control_rate*circular_time)
            {
                circular_success = true;
                count = 0;
            }
            count++;
            info << "Waypoint Tracking > Velocity_x: " << uav_command.velocity_ref[0] << " Veloticy_y: " << uav_command.velocity_ref[1]<< " count: " << count;
            PCOUT(1, GREEN, info.str());
            if(is_detected && circular_success)
            {
                exec_state = TRACKING;
                PCOUT(1, GREEN, "tracking");
                loss_count = max_loss_count;
            }
            break;
        case SEARCH_MOVING:
            // sleep(10);
            //坐标系
            uav_command.header.frame_id = "ENU";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // Move_mode
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
            //无人机按照圆形轨迹飞行
            get_circular_property(31, 40, 5);

            uav_command.velocity_ref[0] = -line_velocity * std::sin(count * angle_increment);
            uav_command.velocity_ref[1] = line_velocity * std::cos(count * angle_increment);
            uav_command.velocity_ref[2] = 0;
            uav_command.position_ref[2] = 2.3;
            //发布的命令ID加1
            //发布降落命令
            //计数器
            if(count > control_rate*circular_time)
            {
                circular_success = true;
            }
            count++;
            info << "Waypoint Tracking > Velocity_x: " << uav_command.velocity_ref[0] << " Veloticy_y: " << uav_command.velocity_ref[1];
            PCOUT(1, GREEN, info.str());
            if(is_detected && circular_success)
            {
                exec_state = TRACKING_MOVING;
                PCOUT(1, GREEN, "tracking");
                loss_count = max_loss_count;
            }
            break;
        case TRACKING:
            if (!is_detected)
            {
                --loss_count;
                if(loss_count < 0)
                    exec_state = RETURN;
                    PCOUT(0, YELLOW, "Return");
            }
            //坐标系
            uav_command.header.frame_id = "BODY";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // 机体系下的速度控制
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY;
            uav_command.velocity_ref[0] = -0.9 * ellipse_det.sight_angle[0];
            uav_command.velocity_ref[1] = 0.9 * ellipse_det.sight_angle[1];
            uav_command.velocity_ref[2] = 0;
            uav_command.position_ref[2] = 0;
            uav_command.yaw_ref = 0.0;
            info << "Find object,Go to the target point > velocity_x: " << uav_command.velocity_ref[0] << " [m/s] "
                    << "velocity_y: " << uav_command.velocity_ref[1] << " [m/s] "
                    << std::endl;
            PCOUT(1, GREEN, info.str());
            if(is_holded && circular_success){
                if(uav_state.position[0] > -9 && uav_state.position[0] < 0)
                    if(uav_state.position[1] > -4 && uav_state.position[1] < 0)
                        exec_state = WAY2;
                if(uav_state.position[0] > -9 && uav_state.position[0] < 0)
                    if(uav_state.position[1] > 0 && uav_state.position[1] < 4)
                        exec_state = WAY3;
            }

            if (std::abs(uav_command.velocity_ref[0]) + std::abs(uav_command.velocity_ref[1]) < 0.03)
            {
                hold_count++;
                hold_lost = 0;
            }
            else{
                hold_count = 0;
                hold_lost++;
            }
            if(hold_lost > HOLD_THRES)
            {
                is_holded = false;
                // PCOUT(1, GREEN, "no hold");
            }
            if(hold_count > HOLD_THRES){
                is_holded = true;
                // PCOUT(1, GREEN, "holded");
            }
            break;
        case TRACKING_MOVING:
            if (!is_detected)
            {
                --loss_count;
                if(loss_count < 0)
                    exec_state = RETURN;
                    PCOUT(0, YELLOW, "Return");
            }
            //坐标系
            uav_command.header.frame_id = "BODY";
            // Move模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // 机体系下的速度控制
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY;
            uav_command.velocity_ref[0] = -2.5 * ellipse_det.sight_angle[0];
            uav_command.velocity_ref[1] = 2 * ellipse_det.sight_angle[1];
            uav_command.velocity_ref[2] = 0;
            uav_command.position_ref[2] = 0;
            uav_command.yaw_ref = 0.0;
            info << "Find object,Go to the target point > velocity_x: " << uav_command.velocity_ref[0] << " [m/s] "
                    << "velocity_y: " << uav_command.velocity_ref[1] << " [m/s] "
                    << std::endl;
            PCOUT(1, GREEN, info.str());
            // if(is_holded){
            //     if(uav_state.position[0] > -9 && uav_state.position[0] < 0)
            //         if(uav_state.position[1] > -4 && uav_state.position[1] < 0)
            //             exec_state = WAY2;
            //     if(uav_state.position[0] > -9 && uav_state.position[0] < 0)
            //         if(uav_state.position[1] > 0 && uav_state.position[1] < 4)
            //             exec_state = WAY3;
            // }

            if (std::abs(uav_command.velocity_ref[0]) + std::abs(uav_command.velocity_ref[1]) < 0.03)
            {
                hold_count++;
                hold_lost = 0;
            }
            else{
                hold_count = 0;
                hold_lost++;
            }
            if(hold_lost > HOLD_THRES)
            {
                is_holded = false;
                // PCOUT(1, GREEN, "no hold");
            }
            if(hold_count > HOLD_THRES){
                is_holded = true;
                // PCOUT(1, GREEN, "holded");
            }
            break;
        case RETURN:
            // return to home 
            uav_command.header.frame_id = "ENU";
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            uav_command.position_ref[0] = 0;
            uav_command.position_ref[1] = 0;
            uav_command.position_ref[2] = 1;
            uav_command.yaw_ref = 0.0;
            cout << GREEN << "return to home" << TAIL << endl;

            sleep(15);
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            cout << GREEN << "landing" << TAIL << endl;

            break;
        }
        uav_command.Command_ID += 1;
        uav_command_pub.publish(uav_command);
        r.sleep();
    }
    return 0;
}
