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
#include <tf/transform_datatypes.h>
#include <ukf_car.h>
//topic 头文件
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/ControlCommand.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;   
Eigen::Vector3f drone_pos;
nav_msgs::Odometry GroundTruth;
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo Detection_raw;          //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::DetectionInfo Detection_ENU;
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_des_prev;
Eigen::Vector3f pos_body_enu_frame;
float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数
float start_point_x,start_point_y,start_point_z;
bool is_detected = false;                                          // 是否检测到目标标志
int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int num_count_vision_regain = 0;                                                      //视觉丢失计数器
int Thres_vision = 0;                                                          //视觉丢失计数器阈值
bool use_ukf;
bool moving_target;
Eigen::VectorXd state_fusion;
Eigen::Vector3f camera_offset;
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

    // 注意这里相机冲下安装 
    pos_body_frame[0] = - Detection_raw.position[1];
    pos_body_frame[1] = - Detection_raw.position[0];
    pos_body_frame[2] = - Detection_raw.position[2]; 

    Eigen::Matrix3f R_Body_to_ENU;

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);

    pos_body_enu_frame = R_Body_to_ENU * pos_body_frame;

    //test, 用于纯控制及逻辑测试
    // pos_body_enu_frame[0] = GroundTruth.pose.pose.position.x - drone_pos[0];
    // pos_body_enu_frame[1] = GroundTruth.pose.pose.position.y - drone_pos[1];
    //若已知降落板高度，则无需使用深度信息。
    pos_body_enu_frame[2] = landing_pad_height - drone_pos[2];

    // Body frame to Inertial frame
    Detection_ENU.frame = 1;
    Detection_ENU.position[0] = drone_pos[0] + pos_body_enu_frame[0];
    Detection_ENU.position[1] = drone_pos[1] + pos_body_enu_frame[1];
    Detection_ENU.position[2] = drone_pos[2] + pos_body_enu_frame[2];
    // Detection_ENU.attitude[2] = _DroneState.attitude[2] + Detection_raw.attitude[2];
    Detection_ENU.attitude[2] = 0.0;

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

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    drone_pos[0] = _DroneState.position[0];
    drone_pos[1] = _DroneState.position[1];
    drone_pos[2] = _DroneState.position[2];
}

void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_landing");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/target", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruth_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //视觉丢失次数阈值
    nh.param<int>("Thres_vision", Thres_vision, 10);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);

    //降落板高度
    nh.param<float>("landing_pad_height", landing_pad_height, 0.0);

    //是否使用UKF
    nh.param<bool>("use_ukf", use_ukf, false);

    //目标运动或静止
    nh.param<bool>("moving_target", moving_target, false);

    //追踪控制参数
    nh.param<float>("kpx_land", kpx_land, 0.1);
    nh.param<float>("kpy_land", kpy_land, 0.1);
    nh.param<float>("kpz_land", kpz_land, 0.1);

    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
    nh.param<float>("start_point_z", start_point_z, 2.0);

    //ukf用于估计目标运动状态，此处假设目标为恒定转弯速率和速度模型（CTRV）模型
    UKF_CAR UKF_CAR;

    //打印现实检查参数
    printf_param();
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
        cin >> start_flag;
    }

    // 起飞
    cout<<"[autonomous_landing]: "<<"Takeoff to predefined position."<<endl;
    Command_Now.Command_ID = 1;
    while( _DroneState.position[2] < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point_x;
        Command_Now.Reference_State.position_ref[1]     = start_point_y;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = 0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
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

    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        if(use_ukf)
        {
            //UKF
            state_fusion = UKF_CAR.Run(Detection_ENU,0.05);

            Eigen::Vector3f target_pos_fusion;

            pos_body_enu_frame[0] = state_fusion[0] - drone_pos[0];
            pos_body_enu_frame[1] = state_fusion[1] - drone_pos[1];
            pos_body_enu_frame[2] = state_fusion[2] - drone_pos[2];
            //pos_body_enu_frame[2] = landing_pad_height - drone_pos[2];
        }

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        printf_result();

        //判断是否满足降落条件
        distance_to_setpoint = pos_body_enu_frame.norm();
        if(distance_to_setpoint < distance_thres)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
            cout <<"[autonomous_landing]: Catched the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
        }else if(!is_detected)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            pos_des_prev[0] = drone_pos[0];
            pos_des_prev[1] = drone_pos[1];
            pos_des_prev[2] = drone_pos[2];
            cout <<"[autonomous_landing]: Lost the Landing Pad. "<< endl;
        }else if(abs(pos_body_enu_frame[2]) < 0.3)
        {
            cout <<"[autonomous_landing]: Reach the lowest height. "<< endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
        }else
        {
            cout <<"[autonomous_landing]: Tracking the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position

            Eigen::Vector3f vel_command;
            if(moving_target)
            {
                vel_command[0] = 1.0 + kpx_land * (pos_body_enu_frame[0] + 0.1);
            }else{
                vel_command[0] = kpx_land * pos_body_enu_frame[0];
            }
            
            vel_command[1] = kpy_land * pos_body_enu_frame[1];
            vel_command[2] = kpz_land * pos_body_enu_frame[2];

            for (int i=0; i<3; i++)
            {
                Command_Now.Reference_State.position_ref[i] = pos_des_prev[i] + vel_command[i]* 0.05;
            }

            // 机体系速度控制有些bug
            // Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            // Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            // Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;   //xy velocity z position

            // Eigen::Vector3f vel_command;
            // vel_command[0] = 1.0 + kpx_land * (pos_body_frame[0] + 0.1);
            // vel_command[1] = kpy_land * pos_body_frame[1];
            // vel_command[2] = kpz_land * pos_body_frame[2];

            // for (int i=0; i<3; i++)
            // {
            //     Command_Now.Reference_State.velocity_ref[i] = vel_command[i];
            // }


            Command_Now.Reference_State.yaw_ref             = 0.0;
            
            for (int i=0; i<3; i++)
            {
                pos_des_prev[i] = Command_Now.Reference_State.position_ref[i];
            }

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
    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    cout << "Detection_raw(pos): " << pos_body_frame[0] << " [m] "<< pos_body_frame[1] << " [m] "<< pos_body_frame[2] << " [m] "<<endl;
    cout << "Detection_raw(yaw): " << Detection_raw.yaw_error/3.1415926 *180 << " [deg] "<<endl;


    if(use_ukf)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Before UKF<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Detection_ENU(pos): " << Detection_ENU.position[0] << " [m] "<< Detection_ENU.position[1] << " [m] "<< Detection_ENU.position[2] << " [m] "<<endl;
        cout << "Detection_ENU(yaw): " << Detection_ENU.attitude[2]/3.1415926 *180 << " [du] "<<endl;
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>After UKF<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "State_fusion(pos):  " << state_fusion[0] << " [m] "<< state_fusion[1] << " [m] "<< state_fusion[2] << " [m] "<<endl;
        cout << "State_fusion(vel):  " << state_fusion[2] << " [m/s] "<<endl;
        cout << "State_fusion(yaw):  " << state_fusion[3]/3.1415926 *180 << " [deg] "<< state_fusion[4]/3.1415926 *180 << " [deg/s] "<<endl;
    }else
    {
        cout << "Detection_ENU(pos): " << Detection_ENU.position[0] << " [m] "<< Detection_ENU.position[1] << " [m] "<< Detection_ENU.position[2] << " [m] "<<endl;
        cout << "Detection_ENU(yaw): " << Detection_ENU.attitude[2]/3.1415926 *180 << " [deg] "<<endl;
    }
        cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
    

    tf::Quaternion quat;
    tf::quaternionMsgToTF(GroundTruth.pose.pose.orientation, quat);
 
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    cout << "Ground_truth(yaw):  " << yaw/3.1415926 *180 << " [deg] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "pos_des: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Thres_vision : "<< Thres_vision << endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "landing_pad_height : "<< landing_pad_height << endl;
    cout << "kpx_land : "<< kpx_land << endl;
    cout << "kpy_land : "<< kpy_land << endl;
    cout << "kpz_land : "<< kpz_land << endl;
    cout << "start_point_x : "<< start_point_x << endl;
    cout << "start_point_y : "<< start_point_y << endl;
    cout << "start_point_z : "<< start_point_z << endl;
}

