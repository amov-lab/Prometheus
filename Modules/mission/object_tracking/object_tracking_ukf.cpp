/***************************************************************************************************************************
 * object_tracking.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.2.11
 *
 * 说明: 目标追踪示例程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及追踪策略
 *      3. 发布上层控制指令 (prometheus_msgs::ControlCommand)
***************************************************************************************************************************/
//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>
#include <ukf_ncv.h>

//topic 头文件
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>

using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::DroneState _DroneState;   
Eigen::Vector3f drone_pos;
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo Detection_raw;          //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::DetectionInfo Detection_ENU;

Eigen::VectorXd state_fusion;
Eigen::Vector3f pos_des_prev;
float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数
bool is_detected = false;                                          // 是否检测到目标标志
int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int num_count_vision_regain = 0;                                                      //视觉丢失计数器
int Thres_vision = 0;                                                          //视觉丢失计数器阈值
//---------------------------------------Track---------------------------------------------
float distance_to_setpoint;
float distance_thres;
Eigen::Vector3f tracking_delta;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();                                                                 //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    Detection_raw = *msg;
    
    if(Detection_raw.detected)
    {
        num_count_vision_regain++;
        num_count_vision_lost = 0;
    }else
    {
        num_count_vision_regain = 0;
        num_count_vision_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(num_count_vision_lost > Thres_vision)
    {
        is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(num_count_vision_regain > Thres_vision)
    {
        is_detected = true;
    }
    // Body frame to Inertial frame
    Detection_ENU.frame = 1;

    Detection_ENU.position[0] = _DroneState.position[0] - Detection_raw.position[1];
    Detection_ENU.position[1] = _DroneState.position[1] - Detection_raw.position[0];
    Detection_ENU.attitude[2] = _DroneState.attitude[2] + Detection_raw.attitude[2];
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    drone_pos[0] = _DroneState.position[0];
    drone_pos[1] = _DroneState.position[1];
    drone_pos[2] = _DroneState.position[2];
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nh("~");
    
    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    // 注意这里为了复用程序使用了/prometheus/target作为话题名字，适用于椭圆、二维码、yolo等视觉算法
    // 故同时只能运行一种视觉识别程序，如果想同时追踪多个目标，这里请修改接口话题的名字
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/target", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //视觉丢失次数阈值
    nh.param<int>("Thres_vision", Thres_vision, 10);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);

    //追踪的前后间隔
    nh.param<float>("tracking_delta_x", tracking_delta[0], 0.0);
    nh.param<float>("tracking_delta_y", tracking_delta[1], 0.0);
    nh.param<float>("tracking_delta_z", tracking_delta[2], 0.0);

    //追踪控制参数
    nh.param<float>("kpx_land", kpx_land, 0.1);
    nh.param<float>("kpy_land", kpy_land, 0.1);
    nh.param<float>("kpz_land", kpz_land, 0.1);
    
    //ukf用于估计目标运动状态，此处假设目标为恒定转弯速率和速度模型（CTRV）模型
    UKF_NCV UKF_NCV;

    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 先读取一些飞控的数据
    for(int i=0;i<10;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    pos_des_prev[0] = drone_pos[0];
    pos_des_prev[1] = drone_pos[1];
    pos_des_prev[2] = drone_pos[2];

    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;

    // 起飞
    cout<<"[object_tracking]: "<<"Takeoff to predefined position."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = 1;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 1;
    Command_Now.Reference_State.position_ref[1]     = 1;
    Command_Now.Reference_State.position_ref[2]     = 2.5;
    Command_Now.Reference_State.yaw_ref             = 0;

    //command_pub.publish(Command_Now);

    //sleep(8.0);

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        state_fusion = UKF_NCV.Run(Detection_ENU,0.05);

        printf_result();

        Eigen::Vector3f target_pos_fusion;

        target_pos_fusion[0] = state_fusion[0];
        target_pos_fusion[1] = state_fusion[1];
        target_pos_fusion[2] = state_fusion[2];

        //判断是否满足降落条件
        distance_to_setpoint = cal_distance_tracking(target_pos_fusion,drone_pos,tracking_delta);      
        if(!is_detected)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            pos_des_prev[0] = drone_pos[0];
            pos_des_prev[1] = drone_pos[1];
            pos_des_prev[2] = drone_pos[2];
            cout <<"[object_tracking]: Lost the Target "<< endl;
        }else 
        {
            cout <<"[object_tracking]: Tracking the Target, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            //Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;   //xy velocity z position
            Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position

            Eigen::Vector3f vel_command;
            vel_command[0] = kpx_land * (state_fusion[0] - drone_pos[0]);
            vel_command[1] = kpy_land * (state_fusion[1] - drone_pos[1]);
            vel_command[2] = kpz_land * (state_fusion[2] - drone_pos[2]);

            for (int i=0; i<3; i++)
            {
                Command_Now.Reference_State.position_ref[i] = pos_des_prev[i] + vel_command[i]* 0.05;
            }
            
            for (int i=0; i<3; i++)
            {
                pos_des_prev[i] = Command_Now.Reference_State.position_ref[i];
            }


            Command_Now.Reference_State.velocity_ref[0]     = state_fusion[3];
            Command_Now.Reference_State.velocity_ref[1]     = state_fusion[4];
            Command_Now.Reference_State.velocity_ref[2]     = state_fusion[5];
            Command_Now.Reference_State.acceleration_ref[0] = 0;
            Command_Now.Reference_State.acceleration_ref[1] = 0;
            Command_Now.Reference_State.acceleration_ref[2] = 0;
            Command_Now.Reference_State.yaw_ref             = 0.0;
        }
        
        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        command_pub.publish(Command_Now);

        rate.sleep();
    }

    return 0;

}

void printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Obeject Tracking<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }

    cout << "Detection_raw(pos): " << Detection_raw.position[0] << " [m] "<< Detection_raw.position[1] << " [m] "<<endl;
    cout << "Detection_raw(yaw): " << Detection_raw.attitude[2]/3.1415926 *180 << " [du] "<<endl;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>UKF State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "State_fusion(pos): " << state_fusion[0] << " [m] "<< state_fusion[1] << " [m] "<<endl;
    cout << "State_fusion(vel): " << state_fusion[2] << " [m/s] "<<endl;
    cout << "State_fusion(yaw): " << state_fusion[3]/3.1415926 *180 << " [du] "<< state_fusion[4]/3.1415926 *180 << " [du/s] "<<endl;
  
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "pos_des: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Thres_vision : "<< Thres_vision << endl;
    cout << "distance_thres : "<< distance_thres << endl;

    cout << "tracking_delta_x : "<< tracking_delta[0] << endl;
    cout << "tracking_delta_y : "<< tracking_delta[1] << endl;
    cout << "tracking_delta_z : "<< tracking_delta[2] << endl;

    cout << "kpx_land : "<< kpx_land << endl;
    cout << "kpy_land : "<< kpy_land << endl;
    cout << "kpz_land : "<< kpz_land << endl;
}


