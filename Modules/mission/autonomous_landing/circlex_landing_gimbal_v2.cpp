/***************************************************************************************************************************
 *
 * Author: Onxming
 *
 ***************************************************************************************************************************/
// ros头文件
#include "prometheus_msgs/gimbal.h"
#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/DroneState.h"
#include "printf_utils.h"

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <prometheus_msgs/GimbalCtl.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <sstream>
#include <Eigen/Eigen>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float pixel_x, pixel_y;
prometheus_msgs::gimbal read_gimbal;
//---------------------------------------Vision---------------------------------------------
// geometry_msgs::Pose pos_target; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

float curr_pos[3];
float curr_vel[3];
std::string drone_mode;
float drone_pitch;
float height_time = 0.0;
int flag_detected = 0; // 是否检测到目标标志

float horizontal_distance = 0.;

//---------------------------------------Track---------------------------------------------
int Num_StateMachine = 0;      // 状态机编号
int Num_StateMachine_Last = 0; // 状态机编号last

float default_pitch;
float kpx_track; //追踪比例系数

float land_move_scale; //降落移动系数
float land_high;
float static_vel_thresh;
float max_vel;
int ignore_error_yaw;
int ignore_error_pitch;

const int yaw_max_count = 10;
int yaw_count = yaw_max_count;
const int horizontal_max_count = 10;
int horizontal_count = horizontal_max_count;
const int vertical_max_count = 10;
int vertical_count = vertical_max_count;
int max_high = 10; // 丢失目标后, 上升的最大高度
float pic_width;
float pic_high;
// 几秒到达最大速度
float vel_smooth_scale = 0.2;
float vel_smooth_thresh = 0.3;
// 目标丢失时速度衰减
float object_lost_vel_weaken;
// 视觉降落计数器

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_now;  //发送给position_control.cpp的命令
prometheus_msgs::ControlCommand Command_past; //发送给position_control.cpp的命令

// 降落状态量
enum State
{
    Horizontal,
    Vertical,
};

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();  //打印各项参数以供检查
void printf_result(); //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

inline float angle2radians(float angle)
{
    return M_PI * angle / 180;
}

prometheus_msgs::DroneState g_dronestate;
void droneStateCb(const prometheus_msgs::DroneState::ConstPtr &msg)
{
    g_dronestate = *msg;
}

void sub_diff_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    flag_detected = msg->data[0];
    pixel_x = msg->data[1] - msg->data[3] / 2;
    pixel_y = msg->data[2] - msg->data[4] / 2;
    pic_width = msg->data[3];
    pic_high = msg->data[4];
}

void sub_gimbaldata_cb(const prometheus_msgs::gimbal::ConstPtr &msg)
{
    read_gimbal.imu0 = msg->imu0;
    read_gimbal.imu1 = msg->imu1;
    read_gimbal.imu2 = msg->imu2;
    read_gimbal.rel0 = msg->rel0;
    read_gimbal.rel1 = msg->rel1;
    read_gimbal.rel2 = msg->rel2;
    read_gimbal.relvel2 = msg->relvel2;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_tracking");
    ros::NodeHandle nh("~");

    std::string path = std::string("/home/amov/Prometheus/circlex_detect_") + std::to_string((int)ros::Time::now().toSec()) + ".log";
    ofstream file(path.c_str());

    // vision_pixel
    ros::Subscriber pos_diff_sub = nh.subscribe<std_msgs::Float32MultiArray>("/prometheus/object_detection/circlex_det", 10, sub_diff_callback);

    // TODO:
    // gimbal_data, 云台数据
    ros::Subscriber gimbaldata_sub = nh.subscribe<prometheus_msgs::gimbal>("/readgimbal", 10, sub_gimbaldata_cb);

    // 获取 无人机ENU下的位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, droneStateCb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // TODO:
    // 手动控制吊舱. 控制初始角度向下
    ros::Publisher gimbal_control_pub = nh.advertise<prometheus_msgs::GimbalCtl>("/GimbalCtl", 10);

    // TODO:
    // 控制吊舱是否接受圆叉检测器的控制
    ros::ServiceClient gimbal_control_flag_client = nh.serviceClient<std_srvs::SetBool>("/prometheus/circlex_gimbal_control");
    gimbal_control_flag_client.waitForExistence();

    // TODO:
    ros::ServiceClient gimbal_change_landing = nh.serviceClient<std_srvs::SetBool>("/prometheus/gimbal/is_land");
    gimbal_change_landing.waitForExistence();

    // 频率 [25Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(25);

    // 降落的吊舱初始角度， 朝下
    prometheus_msgs::GimbalCtl gimbal_control_;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪算法比例参数
    nh.param<float>("kpx_track", kpx_track, 0.25);

    //视觉丢失次数阈值
    // 速度平滑
    nh.param<float>("vel_smooth_scale", vel_smooth_scale, 0.2);
    // 使用速度平滑的阈值
    nh.param<float>("vel_smooth_thresh", vel_smooth_thresh, 0.3);

    //吊舱默认向下的转的角度
    nh.param<float>("default_pitch", default_pitch, -60.);

    // 降落速度
    nh.param<float>("land_move_scale", land_move_scale, 0.2);
    // 自动降落高度
    nh.param<float>("land_high", land_high, 0.2);

    // 目标丢失时速度衰减
    nh.param<float>("object_lost_vel_weaken", object_lost_vel_weaken, 0.5);

    int cv_land_count = 5;
    int cv_land_cnt[5]{0, 0, 0, 0, 0};

    // 忽略的俯仰角误差: 在误差内,判定为无人机已经到达降落板的正上方
    nh.param<int>("ignore_error_pitch", ignore_error_pitch, 5);
    // 静止速度
    nh.param<float>("static_vel_thresh", static_vel_thresh, 0.02);

    nh.param<float>("max_vel", max_vel, 1);

    enum check_flag
    {
        yaw_pitch_ctl = 1,
        roll_pitch_ctl = 2
    } ctl_type;

    //输入1,继续，其他，退出程序
    cout
        << "Please check the parameter and setting，\n 1 for (yaw + pitch) control \n 2 for mode (roll + pitch) control \n else for quit: " << endl;
    std::cin >> ctl_type;

    // 吊舱位置初始化
    // 摄像头向下90度
    std_srvs::SetBool tmp;
    gimbal_control_.pitch = default_pitch;
    gimbal_control_pub.publish(gimbal_control_);
    switch (ctl_type)
    {
    case yaw_pitch_ctl:
        tmp.request.data = false;
        break;

    case roll_pitch_ctl:
        tmp.request.data = true;
        break;
    default:
        return -1;
    }
    gimbal_change_landing.call(tmp);
    gimbal_control_.pitch = 0.;

    // 丢失时使用的数据
    float lost_x_vel, lost_y_vel, lost_z_vel;
    int land_cnt = 0;
    bool cv_land = false;

    constexpr int max_count_vision_lost = 25 * 2; //视觉丢失计数器
    int count_vision_lost;                        //视觉丢失计数器阈值
    count_vision_lost = max_count_vision_lost;

    // 状态初值
    State Now_State = State::Horizontal;
    Command_past.Mode = prometheus_msgs::ControlCommand::Hold;
    Command_now.source = "circlex_landing";
    Command_now.Command_ID = 0;
    Command_past.Reference_State.velocity_ref[0] = 0.;
    Command_past.Reference_State.velocity_ref[1] = 0.;
    Command_past.Reference_State.velocity_ref[2] = 0.;
    Command_past.Reference_State.position_ref[0] = 0.;
    Command_past.Reference_State.position_ref[1] = 0.;
    Command_past.Reference_State.position_ref[2] = 0.;

    while (ros::ok())
    {
        ros::spinOnce();
        // 检查是否处于OFFBOARD模式
        if (g_dronestate.mode != "OFFBOARD")
        {
            PCOUT(-1, TAIL, "Waiting for enable OFFBOARD...");
            continue;
        }

        Command_now.Command_ID++;
        Command_now.header.stamp = ros::Time::now();

        float comm[4]{0, 0, 0, 0};
        Command_now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
        Command_now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
        Command_now.Reference_State.Yaw_Rate_Mode = true;

        std::stringstream ss;
        float r, p, y;
        r = read_gimbal.rel0;
        float gimbal_pitch = read_gimbal.rel1 + drone_pitch;
        float gimbal_yaw = read_gimbal.rel2;
        horizontal_distance = curr_pos[2] * std::tan(M_PI * (90 - p) / 180);

        if (curr_pos[2] < land_high || cv_land)
        {
            Command_now.Mode = prometheus_msgs::ControlCommand::Land;
            // TODO: 吊舱回到home 点
            gimbal_control_.pitch = 0.;
            gimbal_control_.yaw = 0.;
            gimbal_control_.home = 1.;
            gimbal_control_pub.publish(gimbal_control_);
            file.close();
        }
        // 如果没有发现目标, 进入目标寻回
        else if (flag_detected == 0)
        {
            int tmp_cv = 0;
            for (int i = 0; i < cv_land_count; i++)
            {
                tmp_cv += cv_land_cnt[i];
                cv_land_cnt[i] = 0;
            }
            // 距离目标过近,导致找不到目标
            if (tmp_cv == cv_land_count)
            {
                cv_land = true;
                PCOUT(0, GREEN, "5 consecutive detections as X before the target disappears, enter LAND");
            }
            // 目标丢失
            else if (count_vision_lost <= 0) // 进入重新寻回目标模式
            {
                if (g_dronestate.position[2] > max_high)
                {
                    cv_land = true;
                    PCOUT(0, RED, "MISSION FAILED");
                }
                else
                {
                    Command_now.Reference_State.velocity_ref[0] = -lost_x_vel;
                    Command_now.Reference_State.velocity_ref[1] = -lost_x_vel;
                    Command_now.Reference_State.velocity_ref[2] = -lost_z_vel;
                    command_now.Reference_state.yaw_rate_ref = 0;
                    lost_x_vel *= object_lost_vel_weaken;
                    lost_y_vel *= object_lost_vel_weaken;
                    PCOUT(1, YELLOW, "MISSION FAILED");
                }
            }
            // 用于在无人机在频繁的交替丢失目标时稳定飞行
            else
            {
                // 记录目标丢失时数据
                if (count_vision_lost == max_count_vision_lost)
                {
                    lost_x_vel = Command_past.Reference_State.velocity_ref[0];
                    lost_y_vel = Command_past.Reference_State.velocity_ref[1];
                    lost_z_vel = Command_past.Reference_State.velocity_ref[2];
                    lost_z_vel = -std::fmax(std::abs(lost_z_vel), 0.3);
                }
                Command_now.Reference_State.velocity_ref[0] = object_lost_vel_weaken * Command_past.Reference_State.velocity_ref[0];
                Command_now.Reference_State.velocity_ref[1] = object_lost_vel_weaken * Command_past.Reference_State.velocity_ref[1];
                Command_now.Reference_State.velocity_ref[2] = object_lost_vel_weaken * Command_past.Reference_State.velocity_ref[2];
                Command_now.Reference_State.yaw_rate_ref = 0;
                PCOUT(1, YELLOW, "object lost, use past command");
                count_vision_lost--;
            }
        }
        else
        {
            // 重置丢失次数
            count_vision_lost = max_count_vision_lost;
            switch (ctl_type)
            {
            case yaw_pitch_ctl:
                switch (Now_State)
                {
                case State::Horizontal:
                {
                    // 如果无人机在接近降落板的正上方时, 锁定吊舱偏航角, 锁定无人机偏航角对齐吊舱
                    if (std::abs(90 - p) < ignore_error_pitch)
                    {
                        if (horizontal_count > 0)
                        {
                            horizontal_count--;
                        }
                        // 进入垂直下降
                        else
                        {
                            // 1. 关闭吊舱跟随
                            std_srvs::SetBool tmp;
                            tmp.request.data = true;
                            gimbal_control_flag_client.call(tmp);
                            // 2. 锁死patch -90, 锁死yaw
                            gimbal_control_.pitch = -90.;
                            gimbal_control_pub.publish(gimbal_control_);
                            gimbal_control_.pitch = 0.;
                            // 3. yaw 回到0度
                            gimbal_control_.yaw = -y;
                            gimbal_control_pub.publish(gimbal_control_);
                            PCOUT(0, GREEN, "yaw lock, patch lock");
                            Now_State = State::Vertical;
                        }
                    }
                    else // 控yaw, xy速度
                    {
                        // 直接根据gimbal_pitch角度调整前进到速度, 并且在航向角误差越大时，前进到速度越慢
                        Command_now.Reference_State.velocity_ref[0] = kpx_track * (90. - gimbal_pitch) / std::fmax(std::abs(gimbal_yaw) - 3, 1);
                        // TODO: 估计距离
                        Command_now.Reference_State.velocity_ref[1] = 0;
                        Command_now.Reference_State.velocity_ref[2] = 0;
                        Command_now.Reference_State.yaw_rate_ref = -gimbal_yaw;
                        PCOUT(0, GREEN, ">> horizontal control << ");
                    }
                    break;
                }
                case State::Vertical:
                {
                    // float offset = std::max(curr_pos[2] * 0.10, 0.10);
                    float offset = land_move_scale * g_dronestate.position[2];
                    // 垂直观测范围:39.8°(近焦) 到 4.2°(远焦)。
                    // tan(53.2/2)=0.50
                    Command_now.Reference_State.velocity_ref[0] = -(pixel_y / (pic_high * 0.5)) * std::tan(M_PI * (39.8 / 2) / 180) * offset;
                    // 水平观测范围:53.2°(近焦) 到 5.65°(远焦)。
                    // tan(53.2/2)=0.50
                    Command_now.Reference_State.velocity_ref[1] = -(pixel_x / (pic_width * 0.5)) * std::tan(M_PI * (53.2 / 2) / 180) * offset;
                    // ((480 + 640)/2)/10 = 56
                    Command_now.Reference_State.velocity_ref[2] = -offset / std::fmax((90 - p) + (std::abs(pixel_y) + std::abs(pixel_x)) / 56, 1);
                    // comm[2] = -std::fmax(offset, 0.05);
                    Command_now.Reference_State.yaw_rate_ref = 0.;
                    std::stringstream ss;
                    ss << "\n >> high control << "
                       << " pixel_error_x: " << pixel_x
                       << " pixel_error_y: " << pixel_y;
                    PCOUT(1, GREEN, ss.str())
                    break;
                }
                }
                break;
            case roll_pitch_ctl:
            {
                // 直线距离估计
                Eigen::Matrix3d rotation_matrix3;
                rotation_matrix3 = Eigen::AngleAxisd(r * M_PI / 180, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(p * M_PI / 180, Eigen::Vector3d::UnitY());
                Eigen::Vector3d euler_angles = rotation_matrix3.eulerAngles(2, 1, 0);
                y = euler_angles[0] * 180 / M_PI;
                y = r > 0 ? y : (y - 180);
                horizontal_distance = (std::tan(angle2radians(90 - p)) / std::abs(std::tan(angle2radians(90 - p)))) *
                                      curr_pos[2] *
                                      std::sqrt(std::pow(std::tan(angle2radians(90 - p)), 2) + std::pow(std::tan(angle2radians(r)), 2));
                // 靠近时, 不控航向角
                if (std::abs(90 - p) < ignore_error_pitch * 2)
                {
                    // -3 吊舱误差偏移
                    comm[0] = land_move_scale * curr_pos[2] * std::tan(angle2radians(90 - p - 3));
                    comm[1] = land_move_scale * curr_pos[2] * std::tan(angle2radians(r));
                    // -3 容忍误差
                    comm[2] = -land_move_scale * curr_pos[2] / std::fmax(std::abs(std::abs(90 - p) + std::abs(r)) - 3, 1);
                    comm[3] = 0;
                    ss << "\n 近距离模式: ";
                }
                else
                {
                    // comm[0] = kpx_track * horizontal_distance * std::abs(std::cos(angle2radians(y)));
                    comm[0] = kpx_track * horizontal_distance / std::fmax(std::abs(y) - 5, 1);
                    comm[1] = 0;
                    comm[2] = -kpx_track * curr_pos[2] * std::fmin(1 / horizontal_distance * horizontal_distance, 0.1);
                    comm[3] = y / 2;
                    ss << "\n 远距离模式: ";
                }
                break;
            }
            }
            // 视觉降落判断
            land_cnt = (land_cnt + 1) % cv_land_count;
            cv_land_cnt[land_cnt] = flag_detected == 2 ? 1 : 0;

            if (flag_detected == 2 && std::abs(curr_vel[1]) + std::abs(curr_vel[2]) < static_vel_thresh)
            {
                ss << "\n 检测到X且当前速度被时为静止状态, 进入LAND ";
                cv_land = true;
                comm[0] = 0;
                comm[1] = 0;
                comm[2] = 0;
                comm[3] = 0;
            }
        }
        ss << "\nyaw: " << y
           << " pitch: " << p
           << " roll: " << r
           << " drone_pitch: " << drone_pitch
           << " high: " << curr_pos[2]
           << " horizontal_distance: " << horizontal_distance;
        ss << "\nnow      vel >> x_vel: " << curr_vel[0] << " y_vel: " << curr_vel[1] << " z_vel: " << curr_vel[2];
        ss << "\ncommand  vel >> x_vel: " << comm[0] << " y_vel: " << comm[1] << " z_vel: " << comm[2] << " max_vel: " << max_vel;
        for (int i = 0; i < 3; i++)
        {
            Command_now.Reference_State.velocity_ref[i] = comm[i] == 0 ? 0 : (comm[i] / std::abs(comm[i])) * std::fmin(std::abs(comm[i]), max_vel);
        }
        Command_now.Reference_State.yaw_rate_ref = comm[3] / 180.0 * M_PI;
        command_pub.publish(Command_now);
        if (Command_now.Mode == prometheus_msgs::ControlCommand::Land)
            break;
        Command_past = Command_now;
        ss << "\n------------------------------------------------------------------------" << std::endl;
        std::cout << ss.str() << std::endl;
        file << ss.str();
        rate.sleep();
    }
    cout << "land finish, press any key to exit" << endl;
    // 防止程序直接退出
    cin >> ctl_type;
    return 0;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "default_pitch           " << default_pitch << endl;
    cout << "land_move_scale         " << land_move_scale << endl;
    cout << "land_high               " << land_high << endl;
    cout << "ignore_error_pitch      " << ignore_error_pitch << endl;
    cout << "max_count_vision_lost   " << max_count_vision_lost << endl;
    cout << "vel_smooth_scale        " << vel_smooth_scale << endl;
    cout << "vel_smooth_thresh       " << vel_smooth_thresh << endl;
    cout << "object_lost_vel_weaken  " << object_lost_vel_weaken << endl;
    cout << "static_vel_thresh       " << static_vel_thresh << endl;
    cout << "max_vel                 " << max_vel << endl;
}
