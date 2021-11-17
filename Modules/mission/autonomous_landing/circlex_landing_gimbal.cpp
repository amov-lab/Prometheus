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
#include <prometheus_msgs/ControlCommand.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <prometheus_msgs/GimbalCtl.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32MultiArray.h>

#include <sstream>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::GimbalTrackError read_pixel;
prometheus_msgs::gimbal read_gimbal;
//---------------------------------------Vision---------------------------------------------
// geometry_msgs::Pose pos_target; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

geometry_msgs::Point curr_pos;
float height_time = 0.0;
int flag_detected = 0; // 是否检测到目标标志

float horizontal_distance = 0.;

//---------------------------------------Track---------------------------------------------
int Num_StateMachine = 0;      // 状态机编号
int Num_StateMachine_Last = 0; // 状态机编号last

float lost_target_up_vel;
float default_pitch;
float kpx_track; //追踪比例系数

float track_max_vel_x; //追踪最大速度
float track_max_vel_y; //追踪最大速度
float track_max_vel_z; //追踪最大速度

float track_thres_vel_x; //追踪速度死区
float track_thres_vel_y; //追踪速度死区
float track_thres_vel_z; //追踪速度死区

int ignore_error_pixel_for_landing;
int ignore_error_yaw;
int ignore_error_pitch;

int num_count_vision_lost = 0; //视觉丢失计数器
int count_vision_lost;    //视觉丢失计数器阈值
int next_status_count_xy = 10; //进入下一状态的阈值
int next_status_count_h = 10;
int max_high = 10; // 丢失目标后, 上升的最大高度

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_now; //发送给position_control.cpp的命令

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                               //打印各项参数以供检查
void printf_result();                              //打印函数
float satfunc(float data, float Max, float Thres); //限幅函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    curr_pos = msg->pose.position;
}

void sub_diff_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    flag_detected = msg->data[0];
    read_pixel.x = msg->data[1] - msg->data[3] / 2;
    read_pixel.y = msg->data[2] - msg->data[4] / 2;
}

void sub_gimbaldata_cb(const prometheus_msgs::gimbal::ConstPtr &msg)
{
    read_gimbal.imu0 = msg->imu0;
    read_gimbal.imu1 = msg->imu1;
    read_gimbal.imu2 = msg->imu2;
    read_gimbal.rel0 = msg->rel0;
    read_gimbal.rel1 = msg->rel1;
    read_gimbal.rel2 = msg->rel2;
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
    ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 手动控制吊舱. 控制初始角度向下
    ros::Publisher gimbal_control_pub = nh.advertise<prometheus_msgs::GimbalCtl>("/GimbalCtl", 10);

    // 控制吊舱是否接受圆叉检测器的控制
    ros::ServiceClient gimbal_control_flag_client = nh.serviceClient<std_srvs::Empty>("/circlex_gimbal_control");
    gimbal_control_flag_client.waitForExistence();

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(20.0);

    // 降落的吊舱初始角度， 朝下
    prometheus_msgs::GimbalCtl gimbal_control_;

    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪算法比例参数
    nh.param<float>("kpx_track", kpx_track, 1.0);

    //追踪算法最大追踪速度
    //取决与使用场景，室内或者第一次实验时，建议选小一点
    nh.param<float>("track_max_vel_x", track_max_vel_x, 0.5);
    nh.param<float>("track_max_vel_y", track_max_vel_y, 0.5);
    nh.param<float>("track_max_vel_z", track_max_vel_z, 0.5);

    //追踪算法速度死区
    nh.param<float>("track_thres_vel_x", track_thres_vel_x, 0.02);
    nh.param<float>("track_thres_vel_y", track_thres_vel_y, 0.02);
    nh.param<float>("track_thres_vel_z", track_thres_vel_z, 0.02);

    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("count_vision_lost", count_vision_lost, 60);

    // 降落过程中目标丢失时的上升速度
    nh.param<float>("lost_target_up_vel", lost_target_up_vel, 0.5);

    //吊舱默认向下的转的角度
    nh.param<float>("default_pitch", default_pitch, -60.);

    // 忽略的像素误差, 误差内不进行修改, 防止无人其抖动
    nh.param<int>("ignore_error_pixel_for_landing", ignore_error_pixel_for_landing, 20);

    // 忽略的偏航角误差: 在误差内,判定为航向已经对齐到降落板
    nh.param<int>("ignore_error_yaw", ignore_error_yaw, 5);

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

    int flag_command; //机体系FLAG

    // 摄像头向下90度
    gimbal_control_.pitch = default_pitch;
    gimbal_control_pub.publish(gimbal_control_);
    gimbal_control_.pitch = 0.;
    int max_count_vision_lost = count_vision_lost;
    bool is_start = true;

    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        Command_now.Command_ID = comid;
        Command_now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
        Command_now.source = "circlex_dectector"
        comid++;
        std::stringstream ss;
        if (flag_detected == 0)
        {
            Command_now.Mode = prometheus_msgs::ControlCommand::Hold;
            if (count_vision_lost <= 0 && !is_start)
            {
                Command_now.Mode = prometheus_msgs::ControlCommand::Move;
                Command_now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_POS_Z_VEL;
                Command_now.Reference_State.position_ref[0] = 0.;
                Command_now.Reference_State.position_ref[1] = 0.;
                Command_now.Reference_State.velocity_ref[2] = curr_pos.z > max_high ? 0 : lost_target_up_vel;
                ss << " >> object lost << ";
            }
            else if (!is_start)
            {
                count_vision_lost--;
            }
        }
        //如果捕捉到目标
        else
        {
            is_start = false;
            count_vision_lost = max_count_vision_lost;

            /********************************
             * 1. 航向角对齐 
             * 2. 高度保持, 平面对齐, 航向角对齐
             * 3. 高度下降, 平面对齐, 航向角对齐
             ********************************/

            Command_now.Mode = prometheus_msgs::ControlCommand::Move;
            // 判读航向角是否对齐
            float p = read_gimbal.rel1, y = read_gimbal.rel2;
            horizontal_distance = curr_pos.z * std::tan(3.141592653 * (90 - p) / 190);

            bool get_command = false;
            static bool yaw_lock_finsh = false;
            ss << "yaw: " << y << " pitch: " << p << " high: " << curr_pos.z
               << " horizontal_distance: " << horizontal_distance << "\n";
            if (std::abs(y) > ignore_error_yaw && !yaw_lock_finsh) // 只控制yaw, 偏航角
            {
                Command_now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
                Command_now.Reference_State.yaw_ref = -3.141592653 * y / 180;
                Command_now.Reference_State.position_ref[0] = 0;
                Command_now.Reference_State.position_ref[1] = 0;
                Command_now.Reference_State.position_ref[2] = 0;
                get_command = true;
                ss << " >> yaw control <<  ";
            }

            if (!get_command && !yaw_lock_finsh)
            {
                get_command = true;
                if (std::abs(90 - p) > ignore_error_pitch) // 控yaw, xy速度
                {
                    Command_now.Reference_State.Move_mode = 2;
                    Command_now.Reference_State.yaw_ref = -3.141592653 * y / 180;
                    Command_now.Reference_State.velocity_ref[0] = kpx_track * horizontal_distance / 4.;
                    Command_now.Reference_State.velocity_ref[1] = 0;
                    Command_now.Reference_State.position_ref[2] = 0;
                    ss << ">> xy control << "
                       << " vel_x: " << Command_now.Reference_State.velocity_ref[0];
                }
                else // 如果无人机在接近降落板的正上方时, 锁定吊舱偏航角, 锁定无人机偏航角对齐吊舱
                {
                    if (next_status_count_xy > 0)
                    {
                        next_status_count_xy--;
                    }
                    else
                    {
                        // 1. 关闭吊舱跟随
                        std_srvs::Empty tmp;
                        gimbal_control_flag_client.call(tmp);
                        // 2. 锁死patch -90, 锁死yaw
                        gimbal_control_.pitch = -90.;
                        gimbal_control_pub.publish(gimbal_control_);
                        gimbal_control_.pitch = 0.;
                        // 3. yaw 回到0度
                        gimbal_control_.yaw = -y;
                        gimbal_control_pub.publish(gimbal_control_);
                        // gimbal_control_.yawfollow = 1;
                        // Command_now.Reference_State.yaw_ref = -3.141592653 * y / 180;
                        Command_now.Reference_State.yaw_ref = 0;
                        yaw_lock_finsh = true;
                        ss << " >> yaw lock, patch lock << ";
                    }
                }
            }

            // 到达接近降落板的正上方后当成固定朝下的相机使用
            // 降落: 关闭吊舱的圆叉跟随, 将吊舱patch锁死在, 直角向下90度
            if (!get_command)
            {
                if (curr_pos.z > 0.3) // 锁定yaw, xyz点
                {
                    if (next_status_count_h > 0)
                    {
                        next_status_count_h--;
                    }
                    else
                    {
                        // 3. xy定点控制无人机, z速度控制
                        Command_now.Reference_State.Move_mode = 1;

                        // x, y 交换, 机体和相机坐标系x,y正好相反
                        if (std::fabs(read_pixel.y) < ignore_error_pixel_for_landing)
                        {
                            Command_now.Reference_State.position_ref[0] = 0.;
                        }
                        else
                        {
                            Command_now.Reference_State.position_ref[0] = read_pixel.y > 0 ? -0.10 : 0.10;
                        }

                        if (std::fabs(read_pixel.x) < ignore_error_pixel_for_landing)
                        {

                            Command_now.Reference_State.position_ref[1] = 0;
                        }
                        else
                        {
                            Command_now.Reference_State.position_ref[1] = read_pixel.x > 0 ? -0.10 : 0.10;
                        }
                        Command_now.Reference_State.velocity_ref[2] = -curr_pos.z / 4.;
                        Command_now.Reference_State.yaw_ref = 0.;
                        ss << " >> high control << "
                           << " pixel_x: " << read_pixel.x
                           << " pixel_y: " << read_pixel.y
                           << " vel_z: " << Command_now.Reference_State.velocity_ref[2];
                    }
                }
                else
                {
                    Command_now.Mode = prometheus_msgs::ControlCommand::Land;
                    // 吊舱回到home
                    gimbal_control_.pitch = 0.;
                    gimbal_control_.yaw = 0.;
                    gimbal_control_.home = 0.;
                    gimbal_control_pub.publish(gimbal_control_);
                    ss << " >> Into landing status. End of program << ";
                }
            }
            //速度限幅
            Command_now.Reference_State.velocity_ref[0] = satfunc(Command_now.Reference_State.velocity_ref[0], track_max_vel_x, track_thres_vel_x);
            Command_now.Reference_State.velocity_ref[1] = satfunc(Command_now.Reference_State.velocity_ref[1], track_max_vel_y, track_thres_vel_y);
            Command_now.Reference_State.velocity_ref[2] = satfunc(Command_now.Reference_State.velocity_ref[2], track_max_vel_z, track_thres_vel_z);
        }

        //Publish
        command_pub.publish(Command_now);
        if (Command_now.Mode == prometheus_msgs::ControlCommand::Land)
            break;
        rate.sleep();
        std::cout << ss.str() << std::endl;
        std::cout << "********************************************************************************" << std::endl;
    }

    return 0;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "default_pitch : " << default_pitch << endl;
    cout << "lost_target_up_vel: " << lost_target_up_vel << endl;
    cout << "ignore_error_pixel_for_landing: " << ignore_error_pixel_for_landing << endl;
    cout << "ignore_error_yaw: " << ignore_error_yaw << endl;
    cout << "ignore_error_pitch: " << ignore_error_pitch << endl;

    cout << "kpx_track : " << kpx_track << endl;

    cout << "track_max_vel_x : " << track_max_vel_x << endl;
    cout << "track_max_vel_y : " << track_max_vel_y << endl;
    cout << "track_max_vel_z : " << track_max_vel_z << endl;

    cout << "track_thres_vel_x : " << track_thres_vel_x << endl;
    cout << "track_thres_vel_y : " << track_thres_vel_y << endl;
    cout << "track_thres_vel_z : " << track_thres_vel_z << endl;
    cout << "count_vision_lost : " << count_vision_lost << endl;
}

//饱和函数
float satfunc(float data, float Max, float Thres)
{
    if (abs(data) < Thres)
    {
        return 0;
    }
    else if (abs(data) > Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}
