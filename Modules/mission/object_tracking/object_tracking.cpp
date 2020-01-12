/***************************************************************************************************************************
 * object_tracking.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.1.12
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
#include <ukf.h>

//topic 头文件
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::DroneState _DroneState;   
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo target_raw;          //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::DetectionInfo target_fusion;

Eigen::Vector3f drone_pos;
Eigen::Vector3f target_pos_body_raw;
Eigen::Vector3f target_pos_enu_raw;
Eigen::Vector3f target_pos_fusion;

int flag_detected = 0;                                          // 是否检测到目标标志
int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int Thres_vision_lost = 0;                                                          //视觉丢失计数器阈值
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
    prometheus_msgs::DetectionInfo target_body;
    target_body = *msg;

    if(target_body.detected)
    {
        flag_detected = 1;
        num_count_vision_lost = 0;
        
    }else
    {
        num_count_vision_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(num_count_vision_lost > Thres_vision_lost)
    {
        flag_detected = 0;
    }

    target_pos_body_raw[0] = msg->position[0];
    target_pos_body_raw[1] = msg->position[1];
    target_pos_body_raw[2] = msg->position[2];

    // Body frame to Inertial frame
    target_raw = target_body;
    target_raw.frame = 1;

    target_raw.position[0] = _DroneState.position[0] + target_body.position[0];
    target_raw.position[1] = _DroneState.position[1] + target_body.position[1];
    target_raw.position[2] = _DroneState.position[2] + target_body.position[2];
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
 
    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    // 注意这里为了复用程序使用了/prometheus/target作为话题名字，适用于椭圆、二维码、yolo等视觉算法
    // 故同时只能运行一种视觉识别程序，如果想同时追踪多个目标，这里请修改接口话题的名字
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/target", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(20.0);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //视觉丢失次数阈值
    nh.param<int>("Thres_vision_lost", Thres_vision_lost, 20);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);

    //追踪的前后间隔
    nh.param<float>("tracking_delta_x", tracking_delta[0], 0.0);
    nh.param<float>("tracking_delta_y", tracking_delta[1], 0.0);
    nh.param<float>("tracking_delta_z", tracking_delta[2], 0.0);


    //ukf用于估计目标运动状态，此处假设目标为匀加速运动
    
    UKF UKF_NCA(1);

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

    // 无人机未解锁或者未进入offboard模式前，循环等待
    while(_DroneState.armed != true || _DroneState.mode != "OFFBOARD")
    {
        cout<<"[sqaure]: "<<"Please arm and switch to OFFBOARD mode."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

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
    cout<<"[object_tracking]: "<<"Takeoff."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Takeoff;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

    command_pub.publish(Command_Now);

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        Eigen::VectorXd target_fusion = UKF_NCA.Run(target_raw,0.05);

        printf_result();

        // 如果 视觉丢失目标 或者 当前与目标距离小于距离阈值, 则无人机悬停于当前点
        distance_to_setpoint = cal_distance_tracking(target_pos_fusion,drone_pos,tracking_delta);
        if(distance_to_setpoint < distance_thres)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            cout <<"[object_tracking]: Catched the Target, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
        }
        else if(flag_detected == 0)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            cout <<"[object_tracking]: Lost the Target "<< endl;
        }else 
        {
            cout <<"[object_tracking]: Tracking the Target, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;   //xy velocity z position
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
    cout.setf(ios::fixed);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "flag_detected: " <<  flag_detected <<endl;
    cout << "num_count_vision_lost: " <<  num_count_vision_lost <<endl;

    cout << "pos_target: [X Y Z] : " << " " << target_raw.position[0]  << " [m] "<< target_raw.position[1]  <<" [m] "<< target_raw.position[2] <<" [m] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Command: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Thres_vision_lost : "<< Thres_vision_lost << endl;
    cout << "distance_thres : "<< distance_thres << endl;

    cout << "tracking_delta_x : "<< tracking_delta[0] << endl;
    cout << "tracking_delta_y : "<< tracking_delta[1] << endl;
    cout << "tracking_delta_z : "<< tracking_delta[2] << endl;
}


