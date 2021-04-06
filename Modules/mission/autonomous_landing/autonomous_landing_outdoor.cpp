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
#include <iostream>
#include <mission_utils.h>
#include <tf/transform_datatypes.h>
#include <ukf_car.h>
#include "message_utils.h"

//*****************************下面是我改动的地方*********************
//#include "home/nvidia/Prometheus/Modules/control/include/state_from_mavros.h"
//#include "command_to_mavros.h"
//#include "prometheus_control_utils.h"
//#include "message_utils.h"
//*****************************上面是我改动的地方*********************

using namespace std;
using namespace Eigen;

#define LANDPAD_HEIGHT 0.0
#define NODE_NAME "autonomous_landing"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;   
//nav_msgs::Odometry GroundTruth;
Eigen::Matrix3f R_Body_to_ENU;
std_msgs::Bool flag_start;
//---------------------------------------Vision---------------------------------------------
Detection_result landpad_det;
Eigen::Vector3f pos_des_prev;

float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数
float start_point_x,start_point_y,start_point_z;

//*****************************下面是我改动的地方*********************
//int iiii=0;
//*****************************上面是我改动的地方*********************
//int debug_mode;
bool use_ukf;
bool moving_target;
Eigen::VectorXd state_fusion;
Eigen::Vector3f camera_offset;
//---------------------------------------Track---------------------------------------------
float distance_to_setpoint;
float distance_thres;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + DOWN_CAMERA_OFFSET_X;
    landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + DOWN_CAMERA_OFFSET_Y;
    landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + DOWN_CAMERA_OFFSET_Z;

    landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;

    //若已知降落板高度，则无需使用深度信息。
    landpad_det.pos_body_enu_frame[2] = LANDPAD_HEIGHT - _DroneState.position[2];

    landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
    landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
    landpad_det.pos_enu_frame[2] = _DroneState.position[2] + landpad_det.pos_body_enu_frame[2];
    // landpad_det.att_enu_frame[2] = _DroneState.attitude[2] + Detection_raw.attitude[2];
    landpad_det.att_enu_frame[2] = 0.0;

    if(landpad_det.Detection_info.detected)
    {
        landpad_det.num_regain++;
        landpad_det.num_lost = 0;
    }else
    {
        landpad_det.num_regain = 0;
        landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(landpad_det.num_lost > VISION_THRES)
    {
        landpad_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(landpad_det.num_regain > VISION_THRES)
    {
        landpad_det.is_detected = true;
    }

}


void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}

//void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
//{
    //GroundTruth = *msg;
//}

void switch_cb(const std_msgs::Bool::ConstPtr& msg)
{
    flag_start = *msg;
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
    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruth_cb);

    ros::Subscriber switch_sub = nh.subscribe<std_msgs::Bool>("/prometheus/switch/landing", 10, switch_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪距离阈值
//*****************************下面是我改动的地方*********************
   // nh.param<float>("distance_thres", distance_thres, 0.2);
      nh.param<float>("distance_thres", distance_thres, 0.2);
//*****************************上面是我改动的地方*********************

    //是否使用UKF
    nh.param<bool>("use_ukf", use_ukf, false);

    //目标运动或静止
    nh.param<bool>("moving_target", moving_target, false);

    // DEBUG 模式
    //nh.param<int>("debug_mode", debug_mode, 0);

    //追踪控制参数
    nh.param<float>("kpx_land", kpx_land, 0.1);
    nh.param<float>("kpy_land", kpy_land, 0.1);
//*****************************下面是我改动的地方*********************
    //nh.param<float>("kpz_land", kpz_land, 0.1);
    nh.param<float>("kpz_land", kpz_land, 0.01);
//*****************************上面是我改动的地方*********************

    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
//*****************************下面是我改动的地方*********************
   // nh.param<float>("start_point_z", start_point_z, 2.0);
    nh.param<float>("start_point_z", start_point_z, 1.0);
//*****************************上面是我改动的地方*********************

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
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Takeoff to predefined position.");
    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;
    while( _DroneState.position[2] < 0.3)
    {
        /*
	Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(5.0).sleep();
        */
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        //Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point_x;
        Command_Now.Reference_State.position_ref[1]     = start_point_y;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = 0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
       // ros::Duration(3.0).sleep();
        ros::Duration(4.0).sleep();

        ros::spinOnce();
    }

    // 先读取一些飞控的数据
    for(int i=0;i<10;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    pos_des_prev[0] = _DroneState.position[0];
    pos_des_prev[1] = _DroneState.position[1];
    pos_des_prev[2] = _DroneState.position[2];

    ros::Duration(3.0).sleep();


    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        if(use_ukf)
        {
            //UKF
            prometheus_msgs::DetectionInfo Detection_ENU;
            Detection_ENU.position[0] = landpad_det.pos_enu_frame[0];
            Detection_ENU.position[1] = landpad_det.pos_enu_frame[1];
            Detection_ENU.position[2] = landpad_det.pos_enu_frame[2];
            Detection_ENU.attitude[2] = landpad_det.att_enu_frame[2];

            state_fusion = UKF_CAR.Run(Detection_ENU,0.05);

            Eigen::Vector3f target_pos_fusion;

            landpad_det.pos_body_enu_frame[0] = state_fusion[0] - _DroneState.position[0];
            landpad_det.pos_body_enu_frame[1] = state_fusion[1] - _DroneState.position[1];
            landpad_det.pos_body_enu_frame[2] = state_fusion[2] - _DroneState.position[2];
            //landpad_det.pos_body_enu_frame[2] = landing_pad_height - _DroneState.position[2];
        }

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        printf_result();

        //判断是否满足降落条件
        distance_to_setpoint = landpad_det.pos_body_enu_frame.norm();
        if(distance_to_setpoint < distance_thres)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
            //iiii=iiii+1;
            //Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
            cout <<"[autonomous_landing]: Catched the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Catched the Landing Pad.");
        }else if(!landpad_det.is_detected)//((!landpad_det.is_detected)&&(iiii==0))
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            //iiii=iiii+1;
            pos_des_prev[0] = _DroneState.position[0];
            pos_des_prev[1] = _DroneState.position[1];
            pos_des_prev[2] = _DroneState.position[2];
            cout <<"[autonomous_landing]: Lost the Landing Pad. "<< endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Lost the Landing Pad.");
        }else if(abs(landpad_det.pos_body_frame[2]) < 0.3)//(_DroneState.position[2]<0.4)
        {
            cout <<"[autonomous_landing]: Reach the lowest height. "<< endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Reach the lowest height.");
            Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
           // Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
           // ros::Duration(4.0).sleep();
           // landpad_det.pos_body_frame[2]=0;
        }else
        {
            cout <<"[autonomous_landing]: Tracking the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Tracking the Landing Pad.");
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position

            Eigen::Vector3f vel_command;
            if(moving_target)
            {
                vel_command[0] = 1.0 + kpx_land * (landpad_det.pos_body_enu_frame[0] + 0.1);
            }else{
                vel_command[0] = kpx_land * landpad_det.pos_body_enu_frame[0];
            }
            
            vel_command[1] = kpy_land * landpad_det.pos_body_enu_frame[1];
            vel_command[2] = kpz_land * landpad_det.pos_body_enu_frame[2];

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
        Command_Now.source = NODE_NAME;
	command_pub.publish(Command_Now);
        //if (debug_mode == 0)
        //{
            //command_pub.publish(Command_Now);
        //}
        

        rate.sleep();

    }

    return 0;

}


void printf_result()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(landpad_det.is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    cout << "Detection_raw(pos): " << landpad_det.pos_body_frame[0] << " [m] "<< landpad_det.pos_body_frame[1] << " [m] "<< landpad_det.pos_body_frame[2] << " [m] "<<endl;
    cout << "Detection_raw(yaw): " << landpad_det.Detection_info.yaw_error/3.1415926 *180 << " [deg] "<<endl;


    if(use_ukf)
    {
        // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Before UKF<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        // cout << "Detection_ENU(pos): " << Detection_ENU.position[0] << " [m] "<< Detection_ENU.position[1] << " [m] "<< Detection_ENU.position[2] << " [m] "<<endl;
        // cout << "Detection_ENU(yaw): " << Detection_ENU.attitude[2]/3.1415926 *180 << " [du] "<<endl;
        // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>After UKF<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        // cout << "State_fusion(pos):  " << state_fusion[0] << " [m] "<< state_fusion[1] << " [m] "<< state_fusion[2] << " [m] "<<endl;
        // cout << "State_fusion(vel):  " << state_fusion[2] << " [m/s] "<<endl;
        // cout << "State_fusion(yaw):  " << state_fusion[3]/3.1415926 *180 << " [deg] "<< state_fusion[4]/3.1415926 *180 << " [deg/s] "<<endl;
    }else
    {
        cout << "Detection_ENU(pos): " << landpad_det.pos_enu_frame[0] << " [m] "<< landpad_det.pos_enu_frame[1] << " [m] "<< landpad_det.pos_enu_frame[2] << " [m] "<<endl;
        cout << "Detection_ENU(yaw): " << landpad_det.att_enu_frame[2]/3.1415926 *180 << " [deg] "<<endl;
    }
    //if (debug_mode == 1)
    //{
        //cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
        //cout << "Detection_ENU(pos): " << landpad_det.pos_enu_frame[0] << " [m] "<< landpad_det.pos_enu_frame[1] << " [m] "<< landpad_det.pos_enu_frame[2] << " [m] "<<endl;
        //cout << "Detection_ENU(yaw): " << landpad_det.att_enu_frame[2]/3.1415926 *180 << " [deg] "<<endl;
    //}

    //tf::Quaternion quat;
    //tf::quaternionMsgToTF(GroundTruth.pose.pose.orientation, quat);
 
    //double roll, pitch, yaw;//定义存储r\p\y的容器
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    //cout << "Ground_truth(yaw):  " << yaw/3.1415926 *180 << " [deg] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "pos_des: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "kpx_land : "<< kpx_land << endl;
    cout << "kpy_land : "<< kpy_land << endl;
    cout << "kpz_land : "<< kpz_land << endl;
    cout << "start_point_x : "<< start_point_x << endl;
    cout << "start_point_y : "<< start_point_y << endl;
    cout << "start_point_z : "<< start_point_z << endl;
}


