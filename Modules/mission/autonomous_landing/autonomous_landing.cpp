/***************************************************************************************************************************
 * autonomous_landing.cpp
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
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>
#include <ukf.h>

//topic 头文件
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/ControlCommand.h>
#include <geometry_msgs/Point.h>


using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;   
Eigen::Vector3f drone_pos;
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo Detection_raw;          //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::DetectionInfo Detection_ENU;

Eigen::VectorXd state_fusion;
Eigen::Vector3f tracking_delta;

bool is_detected = false;                                          // 是否检测到目标标志
int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int num_count_vision_regain = 0;                                                      //视觉丢失计数器
int Thres_vision = 0;                                                          //视觉丢失计数器阈值
//---------------------------------------Track---------------------------------------------
float distance_to_setpoint;
float distance_thres;
float landing_pad_height;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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

    Detection_ENU.position[0] = _DroneState.position[0] + Detection_raw.position[0];
    Detection_ENU.position[1] = _DroneState.position[1] + Detection_raw.position[1];

    Detection_ENU.attitude[2] = _DroneState.attitude[2] + Detection_raw.attitude[2];
}



void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    drone_pos[0] = _DroneState.position[0];
    drone_pos[1] = _DroneState.position[1];
    drone_pos[2] = _DroneState.position[2];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_landing");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    // 来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/target", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //视觉丢失次数阈值
    nh.param<int>("Thres_vision", Thres_vision, 10);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);

    //降落板高度
    nh.param<float>("landing_pad_height", landing_pad_height, 0.0);

    //追踪的前后间隔
    nh.param<float>("tracking_delta_x", tracking_delta[0], 0.0);
    nh.param<float>("tracking_delta_y", tracking_delta[1], 0.0);
    nh.param<float>("tracking_delta_z", tracking_delta[2], 0.0);


    //ukf用于估计目标运动状态，此处假设目标为恒定转弯速率和速度模型（CTRV）模型
    int model_type = UKF::CAR;
    
    UKF UKF_CAR(model_type);

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
    cout<<"[autonomous_landing]: "<<"Takeoff to predefined position."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = 1;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 5.0;
    Command_Now.Reference_State.yaw_ref             = 0;

    command_pub.publish(Command_Now);

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        state_fusion = UKF_CAR.Run(Detection_ENU,0.05);

        printf_result();

        Eigen::Vector3f target_pos_fusion;

        target_pos_fusion[0] = state_fusion[0];
        target_pos_fusion[1] = state_fusion[1];
        target_pos_fusion[2] = 0.0;

        //判断是否满足降落条件
        distance_to_setpoint = cal_distance_tracking(target_pos_fusion,drone_pos,tracking_delta);
        if(distance_to_setpoint < distance_thres)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
            cout <<"[autonomous_landing]: Catched the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
        }

        if(!is_detected)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            cout <<"[autonomous_landing]: Lost the Landing Pad. "<< endl;
        }else
        {
            cout <<"[autonomous_landing]: Tracking the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;   //xy velocity z position
            Command_Now.Reference_State.position_ref[0]     = state_fusion[0];
            Command_Now.Reference_State.position_ref[1]     = state_fusion[1];
            Command_Now.Reference_State.position_ref[2]     = landing_pad_height - tracking_delta[2];
            Command_Now.Reference_State.velocity_ref[0]     = state_fusion[2]*cos(state_fusion[3]);
            Command_Now.Reference_State.velocity_ref[1]     = state_fusion[2]*sin(state_fusion[3]);
            Command_Now.Reference_State.velocity_ref[2]     = 0;
            Command_Now.Reference_State.acceleration_ref[0] = 0;
            Command_Now.Reference_State.acceleration_ref[1] = 0;
            Command_Now.Reference_State.acceleration_ref[2] = 0;
            Command_Now.Reference_State.yaw_ref             = state_fusion[3];
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

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Thres_vision : "<< Thres_vision << endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "landing_pad_height : "<< landing_pad_height << endl;
    cout << "tracking_delta_x : "<< tracking_delta[0] << endl;
    cout << "tracking_delta_y : "<< tracking_delta[1] << endl;
    cout << "tracking_delta_z : "<< tracking_delta[2] << endl;
}

