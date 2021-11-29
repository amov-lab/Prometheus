/***************************************************************************************************************************
 *
 * Author: Onxming
 *
***************************************************************************************************************************/
//ros头文件
#include <ros/ros.h>

//topic 头文件
#include "prometheus_msgs/GimbalTrackError.h"
#include "prometheus_msgs/gimbal.h"
#include <iostream>
#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/DroneState.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <prometheus_msgs/GimbalCtl.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <sstream>

using namespace std;

ofstream file("/home/amov/Prometheus/circlex_detect.log");
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::GimbalTrackError read_pixel;
prometheus_msgs::gimbal read_gimbal;
//---------------------------------------Vision---------------------------------------------
// geometry_msgs::Pose pos_target; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

float curr_pos[3];
float curr_vel[3];
// float high_error = 0.0;
float height_time = 0.0;
int flag_detected = 0; // 是否检测到目标标志

float horizontal_distance = 0.;

//---------------------------------------Track---------------------------------------------
int Num_StateMachine = 0;      // 状态机编号
int Num_StateMachine_Last = 0; // 状态机编号last

float default_pitch;
float kpx_track; //追踪比例系数

float land_move_scale; //降落移动系数
int ignore_error_yaw;
int ignore_error_pitch;

int max_count_vision_lost = 0; //视觉丢失计数器
int count_vision_lost;         //视觉丢失计数器阈值
const int yaw_max_count = 10;
int yaw_count = yaw_max_count;
const int horizontal_max_count = 10;
int horizontal_count = horizontal_max_count;
const int vertical_max_count = 10;
int vertical_count = vertical_max_count;
int max_high = 10; // 丢失目标后, 上升的最大高度
int hz = 25;       //循环频率
float pic_width;
float pic_high;
// 几秒到达最大速度
float vel_smooth_scale = 0.2;
float vel_smooth_thresh = 0.3;
// 目标丢失时速度衰减
float object_lost_vel_weaken;
float object_lost_find_duration;

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_now;  //发送给position_control.cpp的命令
prometheus_msgs::ControlCommand Command_past; //发送给position_control.cpp的命令

// 降落状态量
enum State
{
    Horizontal,
    Vertical,
    Landing
};

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();  //打印各项参数以供检查
void printf_result(); //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void pos_cb(const prometheus_msgs::DroneState::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    curr_pos[0] = msg->position[0];
    curr_pos[1] = msg->position[1];
    // curr_pos[2] = msg->position[2];
    curr_pos[2] = msg->rel_alt;
    curr_vel[0] = msg->velocity[0];
    curr_vel[1] = msg->velocity[1];
    curr_vel[2] = msg->velocity[2];
    // high_error = msg->position[2] - msg->rel_alt;
}

void sub_diff_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    flag_detected = msg->data[0];
    read_pixel.x = msg->data[1] - msg->data[3] / 2;
    read_pixel.y = msg->data[2] - msg->data[4] / 2;
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

void generate_com(int Move_mode, float state_desired[4])
{
    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if (Move_mode == prometheus_msgs::PositionReference::XYZ_ACC)
    {
        cout << "ACC control not support yet." << endl;
    }

    if ((Move_mode & 0b10) == 0) //xy channel
    {
        Command_now.Reference_State.position_ref[0] = state_desired[0];
        Command_now.Reference_State.position_ref[1] = state_desired[1];
        Command_now.Reference_State.velocity_ref[0] = 0;
        Command_now.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_now.Reference_State.position_ref[0] = 0;
        Command_now.Reference_State.position_ref[1] = 0;
        Command_now.Reference_State.velocity_ref[0] = state_desired[0];
        Command_now.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if ((Move_mode & 0b01) == 0) //z channel
    {
        Command_now.Reference_State.position_ref[2] = state_desired[2];
        Command_now.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_now.Reference_State.position_ref[2] = 0;
        Command_now.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_now.Reference_State.acceleration_ref[0] = 0;
    Command_now.Reference_State.acceleration_ref[1] = 0;
    Command_now.Reference_State.acceleration_ref[2] = 0;

    Command_now.Reference_State.yaw_ref = state_desired[3] / 180.0 * M_PI;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_tracking");
    ros::NodeHandle nh("~");

    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   orientation.w 用作标志位 1代表识别到目标 0代表丢失目标
    // 注意这里为了复用程序使用了/vision/target作为话题名字，适用于椭圆、二维码、yolo等视觉算法
    // 故同时只能运行一种视觉识别程序，如果想同时追踪多个目标，这里请修改接口话题的名字
    //ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::Pose>("/vision/target", 10, vision_cb);
    //vision_pixel
    ros::Subscriber pos_diff_sub = nh.subscribe<std_msgs::Float32MultiArray>("/prometheus/object_detection/circlex_det", 10, sub_diff_callback);

    //gimbal_data, 云台数据
    ros::Subscriber gimbaldata_sub = nh.subscribe<prometheus_msgs::gimbal>("/readgimbal", 10, sub_gimbaldata_cb);

    // 获取 无人机ENU下的位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, pos_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 手动控制吊舱. 控制初始角度向下
    ros::Publisher gimbal_control_pub = nh.advertise<prometheus_msgs::GimbalCtl>("/GimbalCtl", 10);

    // 控制吊舱是否接受圆叉检测器的控制
    ros::ServiceClient gimbal_control_flag_client = nh.serviceClient<std_srvs::SetBool>("/circlex_gimbal_control");
    gimbal_control_flag_client.waitForExistence();

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(hz);

    // 降落的吊舱初始角度， 朝下
    prometheus_msgs::GimbalCtl gimbal_control_;

    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪算法比例参数
    nh.param<float>("kpx_track", kpx_track, 0.25);

    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("max_count_vision_lost", max_count_vision_lost, 30);

    // 速度平滑
    nh.param<float>("vel_smooth_scale", vel_smooth_scale, 0.2);
    // 使用速度平滑的阈值
    nh.param<float>("vel_smooth_thresh", land_move_scale, 0.3);

    //吊舱默认向下的转的角度
    nh.param<float>("default_pitch", default_pitch, -60.);

    // 降落速度
    nh.param<float>("land_move_scale", land_move_scale, 0.2);

    // 目标丢失时速度衰减
    nh.param<float>("object_lost_vel_weaken", object_lost_vel_weaken, 0.5);

    // 目标丢失时速度衰减
    nh.param<float>("object_lost_find_duration", object_lost_find_duration, 1);

    // 忽略的俯仰角误差: 在误差内,判定为无人机已经到达降落板的正上方
    nh.param<int>("ignore_error_pitch", ignore_error_pitch, 5);

    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;

    if (check_flag != 1)
    {
        return -1;
    }

    // 丢失时使用的数据
    float lost_x_vel, lost_y_vel, lost_z_vel, lost_yaw;
    int lost_find_count = hz * object_lost_find_duration;
    float smooth_count[3] = {1, 1, 1};

    // 摄像头向下90度
    gimbal_control_.pitch = default_pitch;
    gimbal_control_pub.publish(gimbal_control_);
    gimbal_control_.pitch = 0.;
    count_vision_lost = max_count_vision_lost;
    State Now_State = State::Horizontal;
    Command_past.Mode = prometheus_msgs::ControlCommand::Hold;
    Command_past.Reference_State.velocity_ref[0] = 0.;
    Command_past.Reference_State.velocity_ref[1] = 0.;
    Command_past.Reference_State.velocity_ref[2] = 0.;
    Command_past.Reference_State.position_ref[0] = 0.;
    Command_past.Reference_State.position_ref[1] = 0.;
    Command_past.Reference_State.position_ref[2] = 0.;
    float p_past, y_past;
    for (int i = 0; i < 50; i++)
    {
        ros::spinOnce();
        p_past = read_gimbal.rel1, y_past = read_gimbal.rel2;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        float p = std::fabs(read_gimbal.rel1 - p_past) > 10 ? p_past : read_gimbal.rel1;
        float y = std::fabs(read_gimbal.rel2 - y_past) > 10 ? y_past : read_gimbal.rel2;
        p_past = p;
        y_past = y;

        Command_now = Command_past;

        float comm[4]{0, 0, 0, 0};
        Command_now.Command_ID = comid;
        Command_now.header.stamp = ros::Time::now();
        Command_now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
        Command_now.source = "circlex_dectector";
        comid++;

        std::stringstream ss;
        if (Command_past.Mode == prometheus_msgs::ControlCommand::Land)
        {
            ss << "\n >> landing << ";
        }
        else if (flag_detected == 0)
        {
            // TODO 速度衰减, 配置为可变参数
            if (Command_past.Mode == prometheus_msgs::ControlCommand::Hold)
            {
                ss << "\n >> Hold << ";
            }
            else if (count_vision_lost <= 0)
            {
                if (lost_find_count <= 0 || curr_pos[2] > max_high)
                    Command_now.Mode = prometheus_msgs::ControlCommand::Land;
                else
                {
                    Command_now.Reference_State.velocity_ref[0] = -lost_x_vel;
                    Command_now.Reference_State.velocity_ref[1] = -lost_y_vel;
                    Command_now.Reference_State.velocity_ref[2] = -lost_z_vel;
                    Command_now.Reference_State.yaw_ref = -lost_yaw;
                    lost_yaw = 0;
                    lost_x_vel *= 0.99;
                    lost_y_vel *= 0.99;
                    ss << "\n>> object lost, back << ";
                    lost_find_count--;
                }
            }
            else
            {
                if (count_vision_lost == max_count_vision_lost)
                {
                    lost_x_vel = Command_past.Reference_State.velocity_ref[0];
                    lost_y_vel = Command_past.Reference_State.velocity_ref[1];
                    lost_z_vel = Command_past.Reference_State.velocity_ref[2];
                    lost_x_vel = (lost_x_vel / std::abs(lost_x_vel)) * std::fmax(std::abs(lost_x_vel), 0.2);
                    lost_y_vel = (lost_y_vel / std::abs(lost_y_vel)) * std::fmax(std::abs(lost_y_vel), 0.2);
                    lost_z_vel = (lost_z_vel / std::abs(lost_z_vel)) * std::fmax(std::abs(lost_z_vel), 0.2);
                    lost_yaw = Command_now.Reference_State.yaw_ref;
                    lost_find_count = hz * object_lost_find_duration;
                }
                Command_now.Reference_State.velocity_ref[0] *= object_lost_vel_weaken;
                Command_now.Reference_State.velocity_ref[1] *= object_lost_vel_weaken;
                Command_now.Reference_State.velocity_ref[2] *= object_lost_vel_weaken;
                ss << "\n object lost, use past command ! ";
                Command_now.Reference_State.yaw_ref = object_lost_vel_weaken;
                count_vision_lost--;
                // 丢失旋转角度累加
                lost_yaw += lost_yaw * object_lost_vel_weaken;
            }
        }
        //如果捕捉到目标
        else
        {
            // 重置丢失次数
            count_vision_lost = max_count_vision_lost;
            Command_now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_now.Reference_State.Move_frame = prometheus_msgs::PositionReference::XYZ_VEL;
            // 判读航向角是否对齐
            horizontal_distance = curr_pos[2] * std::tan(3.141592653 * (90 - p) / 180);
            ss << "\nyaw: " << y << " pitch: " << p << " yaw_relvel: " << read_gimbal.relvel2 << " high: " << curr_pos[2]
               << " horizontal_distance: " << horizontal_distance;

            switch (Now_State)
            {
            case State::Horizontal:
                // 如果无人机在接近降落板的正上方时, 锁定吊舱偏航角, 锁定无人机偏航角对齐吊舱
                if (std::abs(90 - p) < ignore_error_pitch)
                {
                    if (horizontal_count > 0)
                    {
                        horizontal_count--;
                    }
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
                        // gimbal_control_.yawfollow = 1;
                        ss << "\n >> yaw lock, patch lock << ";
                        Now_State = State::Vertical;
                    }
                }
                else // 控yaw, xy速度
                {
                    comm[0] = kpx_track * horizontal_distance;
                    //  * std::abs(std::cos(-y));
                    comm[1] = 0;
                    comm[2] = 0;
                    comm[3] = -y;
                    ss << "\n>> horizontal control << ";
                }
                break;
            case State::Vertical:
                // 使用真实高度
                if (curr_pos[2] > 0.3) // 锁定yaw, xyz点
                {
                    // 3. xy定点控制无人机, z速度控制

                    // float offset = std::max(curr_pos[2] * 0.10, 0.10);
                    float offset = land_move_scale * curr_pos[2];
                    // 垂直观测范围:39.8°(近焦) 到 4.2°(远焦)。
                    // tan(53.2/2)=0.50
                    comm[0] = -(read_pixel.y / (pic_high * 0.5)) * std::tan(3.141592653 * (39.8 / 2) / 180) * offset * 3;
                    // 水平观测范围:53.2°(近焦) 到 5.65°(远焦)。
                    // tan(53.2/2)=0.50
                    comm[1] = -(read_pixel.x / (pic_width * 0.5)) * std::tan(3.141592653 * (53.2 / 2) / 180) * offset * 3;

                    comm[2] = -offset;
                    // comm[2] = -std::fmax(offset, 0.1);
                    comm[3] = 0.;
                    ss << "\n >> high control << "
                       << " pixel_error_x: " << read_pixel.x
                       << " pixel_error_y: " << read_pixel.y;
                }
                else
                {
                    if (vertical_count > 0)
                    {
                        vertical_count--;
                    }
                    else
                    {
                        Now_State = State::Landing;
                        file.close();
                    }
                }
                break;
            case State::Landing:
                Command_now.Mode = prometheus_msgs::ControlCommand::Land;
                // 吊舱回到home
                gimbal_control_.pitch = 0.;
                gimbal_control_.yaw = 0.;
                gimbal_control_.home = 1.;
                gimbal_control_pub.publish(gimbal_control_);
                break;
            }
            ss << "\n now      vel : x_vel " << curr_vel[0] << " y_vel " << curr_vel[1] << " z_vel " << curr_vel[2];
            ss << "\n   command_vel: x_vel " << comm[0] << " y_vel " << comm[1] << " z_vel " << comm[2];
            // 数据平滑, 防止猛冲
            for (int i = 0; i < 3; i++)
            {
                // 加速时平滑
                float delta = curr_vel[i] - comm[i];
                if (delta < 0 && std::abs(delta) > vel_smooth_thresh)
                {
                    comm[i] = comm[i] / std::abs(comm[i]) * std::max(std::abs(delta * vel_smooth_scale), vel_smooth_scale) + curr_vel[i];
                }
            }
            ss << "\n after smooth : x_vel " << comm[0] << " y_vel " << comm[1] << " z_vel " << comm[2];
            generate_com(Command_now.Reference_State.Move_mode, comm);
        }
        //Publish
        command_pub.publish(Command_now);
        Command_past = Command_now;
        rate.sleep();
        ss << "\n------------------------------------------------------------------------" << std::endl;
        std::cout << ss.str() << std::endl;
        file << ss.str();
    }
    return 0;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "default_pitch           " << default_pitch << endl;
    cout << "land_move_scale         " << land_move_scale << endl;
    cout << "ignore_error_pitch      " << ignore_error_pitch << endl;
    cout << "max_count_vision_lost   " << max_count_vision_lost << endl;
    cout << "vel_smooth_scale        " << vel_smooth_scale << endl;
    cout << "vel_smooth_thresh       " << vel_smooth_thresh << endl;
    cout << "object_lost_vel_weaken  " << object_lost_vel_weaken << endl;
}
