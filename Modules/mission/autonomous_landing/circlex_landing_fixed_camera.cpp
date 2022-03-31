/***************************************************************************************************************************
 *
 * Author: Onxming
 *
 ***************************************************************************************************************************/
// ros头文件
#include <ros/ros.h>

// topic 头文件
#include <iostream>
#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/DroneState.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <sstream>
#include <Eigen/Eigen>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float pixel_x, pixel_y;
//---------------------------------------Vision---------------------------------------------
// geometry_msgs::Pose pos_target; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

float curr_pos[3];
float curr_vel[3];
std::string drone_mode;
float drone_pitch;
float height_time = 0.0;
int flag_detected = 0; // 是否检测到目标标志

//---------------------------------------Track---------------------------------------------
float kpx_track; //追踪比例系数

float land_move_scale; //降落移动系数
float x_move_scale;
float y_move_scale;
float land_high;
float static_vel_thresh;
float max_vel;
bool is_gps = false;

int max_count_vision_lost = 0; //视觉丢失计数器
int count_vision_lost;         //视觉丢失计数器阈值
int max_high = 10;             // 丢失目标后, 上升的最大高度
int hz = 60;                   //循环频率
float pic_width;
float pic_high;
// 几秒到达最大速度
float vel_smooth_scale = 0.2;
float vel_smooth_thresh = 0.3;
// 目标丢失时速度衰减
float object_lost_vel_weaken;
// 视觉降落计数器
int cv_land_count;
int cv_land_cnt[5]{0, 0, 0, 0, 0};

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_now;  //发送给position_control.cpp的命令
prometheus_msgs::ControlCommand Command_past; //发送给position_control.cpp的命令

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param(); //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void pos_cb(const prometheus_msgs::DroneState::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    curr_pos[0] = msg->position[0];
    curr_pos[1] = msg->position[1];
    if (is_gps)
    {
        curr_pos[2] = msg->rel_alt;
    }
    else
    {
        curr_pos[2] = msg->position[2];
    }
    curr_vel[0] = msg->velocity[0];
    curr_vel[1] = msg->velocity[1];
    curr_vel[2] = msg->velocity[2];
    drone_mode = msg->mode;
    drone_pitch = msg->attitude[1] * 180 / M_PI;
    // high_error = msg->position[2] - msg->rel_alt;
}

void sub_diff_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    flag_detected = msg->data[0];
    pixel_x = msg->data[1] - msg->data[3] / 2;
    pixel_y = msg->data[2] - msg->data[4] / 2;
    pic_width = msg->data[3];
    pic_high = msg->data[4];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "circlex_landing_fixed_camera");
    ros::NodeHandle nh("~");

    std::string path = std::string("/home/amov/Prometheus/circlex_detect_") + std::to_string((int)ros::Time::now().toSec()) + ".log";
    ofstream file(path.c_str());
    // vision_pixel
    ros::Subscriber pos_diff_sub = nh.subscribe<std_msgs::Float32MultiArray>("/prometheus/object_detection/circlex_det", 10, sub_diff_callback);

    // 获取 无人机ENU下的位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, pos_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(hz);

    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("max_count_vision_lost", max_count_vision_lost, 30);

    // 速度平滑
    nh.param<float>("vel_smooth_scale", vel_smooth_scale, 0.2);
    // 使用速度平滑的阈值
    nh.param<float>("vel_smooth_thresh", vel_smooth_thresh, 0.3);

    // 降落速度
    nh.param<float>("land_move_scale", land_move_scale, 0.2);
        // 降落速度
    nh.param<float>("x_move_scale", x_move_scale, 0.2);
        // 降落速度
    nh.param<float>("y_move_scale", y_move_scale, 0.2);

    // 自动降落高度
    nh.param<float>("land_high", land_high, 0.2);

    // 目标丢失时速度衰减
    nh.param<float>("object_lost_vel_weaken", object_lost_vel_weaken, 0.5);

    nh.param<int>("cv_land_count", cv_land_count, 5);

    // 静止速度
    nh.param<float>("static_vel_thresh", static_vel_thresh, 0.02);

    nh.param<float>("max_vel", max_vel, 1);

    // 是否室外
    nh.param<bool>("is_gps", is_gps, false);

    //打印现实检查参数
    printf_param();

    int check_flag;
    cout << "Please check the parameter and setting，\n 1 continue \n else for quit: " << endl;
    try
    {
        std::cin >> check_flag;
        if (check_flag != 1)
            return -0;
    }
    catch (std::exception)
    {
        cout << "Please input number" << std::endl;
        return -1;
    }

    // 丢失时使用的数据
    float lost_x_vel, lost_y_vel, lost_z_vel, lost_yaw;
    int land_cnt = 0;
    bool cv_land = false;

    count_vision_lost = max_count_vision_lost;
    Command_past.Mode = prometheus_msgs::ControlCommand::Hold;
    Command_past.Reference_State.velocity_ref[0] = 0.;
    Command_past.Reference_State.velocity_ref[1] = 0.;
    Command_past.Reference_State.velocity_ref[2] = 0.;
    Command_past.Reference_State.position_ref[0] = 0.;
    Command_past.Reference_State.position_ref[1] = 0.;
    Command_past.Reference_State.position_ref[2] = 0.;
    for (int i = 0; i < 50; i++)
    {
        ros::spinOnce();
    }

    while (ros::ok())
    {
        ros::spinOnce();
        float comm[4]{0, 0, 0, 0};
        Command_now.Command_ID = comid;
        Command_now.header.stamp = ros::Time::now();
        Command_now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
        Command_now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
        Command_now.Reference_State.Yaw_Rate_Mode = true;
        Command_now.source = "circlex_dectector";
        comid++;

        std::stringstream ss;
        if ((curr_pos[2] < land_high || cv_land) && drone_mode == "OFFBOARD")
        {
            Command_now.Mode = prometheus_msgs::ControlCommand::Land;
            file.close();
        }
        else if (flag_detected == 0)
        {
            if (drone_mode != "OFFBOARD")
            {
                ss << "\n ready into offboard ....";
            }
            else
            {
                int tmp_cv = 0;
                for (int i = 0; i < cv_land_count; i++)
                {
                    tmp_cv += cv_land_cnt[i];
                    cv_land_cnt[i] = 0;
                }
                if (tmp_cv == cv_land_count)
                {
                    cv_land = true;
                    ss << "\n 目标消失前5次连续检测为X, 进入LAND";
                    comm[0] = 0;
                    comm[1] = 0;
                    comm[2] = 0;
                    comm[3] = 0;
                }
                else if (count_vision_lost <= 0) // 进入重新寻回目标模式
                {
                    if (curr_pos[2] > max_high)
                    {
                        ss << "\n 目标丢失后达到最大高度, 进入LAND";
                        cv_land = true;
                    }
                    else
                    {
                        comm[0] = -lost_x_vel;
                        comm[1] = -lost_y_vel;
                        comm[2] = -lost_z_vel;
                        comm[3] = -lost_yaw;
                        lost_yaw = 0;
                        lost_x_vel *= object_lost_vel_weaken;
                        lost_y_vel *= object_lost_vel_weaken;
                        ss << "\n>> object lost, back << ";
                    }
                }
                else
                {
                    // 记录目标丢失时数据
                    if (count_vision_lost == max_count_vision_lost)
                    {
                        lost_x_vel = Command_past.Reference_State.velocity_ref[0];
                        lost_y_vel = Command_past.Reference_State.velocity_ref[1];
                        lost_z_vel = Command_past.Reference_State.velocity_ref[2];
                        lost_z_vel = -std::fmax(std::abs(lost_z_vel), 0.3);
                        lost_yaw = Command_past.Reference_State.yaw_ref;
                    }
                    comm[0] = object_lost_vel_weaken * Command_past.Reference_State.velocity_ref[0];
                    comm[1] = object_lost_vel_weaken * Command_past.Reference_State.velocity_ref[1];
                    comm[2] = object_lost_vel_weaken * Command_past.Reference_State.velocity_ref[2];
                    ss << "\n object lost, use past command ! ";
                    comm[3] = 0;
                    count_vision_lost--;
                }
            }
        }
        else
        {
            // 重置丢失次数
            count_vision_lost = max_count_vision_lost;
            // 使用真实高度
            // float offset = std::max(curr_pos[2] * 0.10, 0.10);
            comm[0] = -pixel_y * x_move_scale * curr_pos[2] / 100;
            comm[1] = -pixel_x * y_move_scale * curr_pos[2] / 100;
            comm[2] = -land_move_scale * curr_pos[2] / std::fmax((comm[1] +comm[0]) * 10,1);
            comm[3] = 0.;
            ss << "\n"
               << "pixel_error_x: " << pixel_x
               << " pixel_error_y: " << pixel_y
               << " curr_pos: " << curr_pos[2] << " ";
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
        ss << "\nnow      vel >> x_vel: " << curr_vel[0] << " y_vel: " << curr_vel[1] << " z_vel: " << curr_vel[2] << " hight: " << curr_pos[2];
        ss << "\ncommand  vel >> x_vel: " << comm[0] << " y_vel: " << comm[1] << " z_vel: " << comm[2] << " yaw_rate: " << Command_now.Reference_State.yaw_rate_ref << " max_vel: " << max_vel;
        for (int i = 0; i < 3; i++)
        {
            Command_now.Reference_State.velocity_ref[i] = comm[i] == 0 ? 0 : (comm[i] / std::abs(comm[i])) * std::fmin(std::abs(comm[i]), max_vel);
        }
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
    cin >> check_flag;
    return 0;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "land_move_scale         " << land_move_scale << endl;
    cout << "land_high               " << land_high << endl;
    cout << "max_count_vision_lost   " << max_count_vision_lost << endl;
    cout << "vel_smooth_scale        " << vel_smooth_scale << endl;
    cout << "vel_smooth_thresh       " << vel_smooth_thresh << endl;
    cout << "object_lost_vel_weaken  " << object_lost_vel_weaken << endl;
    cout << "static_vel_thresh       " << static_vel_thresh << endl;
    cout << "max_vel                 " << max_vel << endl;
}
