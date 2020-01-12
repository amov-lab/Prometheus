/***************************************************************************************************************************
 * autonomous_landing.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.12.25
 *
 * 说明: 基于单目摄像头的自主降落程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. UKF
 *      3. 追踪算法及降落策略
 *      4. 发布上层控制指令
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <ukf.h>
#include <iostream>

//topic 头文件
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/ControlCommand.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int Num_StateMachine = 0;                                       //状态机编号
int Num_StateMachine_Last = 0;                                  //上一时刻 状态机编号
int comid = 1;                                                  //Command_Now的id号
prometheus_msgs::DroneState _DroneState;                        //无人机位置等状态
//-----------------------------------------目标检测及状态估计----------------------------------------------------


geometry_msgs::Point relative_position;                         //机体固连坐标系下 降落板的位置
prometheus_msgs::DetectionInfo target_raw;                      //视觉检测的原始信息 
prometheus_msgs::DetectionInfo target_raw_inertial;             //目标在xy平面的位置,yaw,惯性系,用于UKF
VectorXd _x_ukf;                                                //UKF状态量

bool is_detected = false;                                       //视觉FLAG 是否识别到目标 1代表能识别到目标，0代表不能
int num_count_vision_lost = 0;
int Thres_vision_lost;

float relative_yaw;                                             //降落板与无人机的相对偏航角 单位:rad
float relative_yaw_last;
float height_on_pad;                                            //降落板高度
//-----------------------------------------降落控制相关----------------------------------------------------
float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数
float Max_velx_land, Max_vely_land, Max_velz_land;                                //控制参数 - 最大值限幅
float Thres_velx_land, Thres_vely_land, Thres_velz_land;                          //控制参数 - 死区限幅

float distance_pad;                                                               //无人机与降落板中心的距离
float Thres_distance_land,Thres_count_land;                                       //控制参数 - 阈值（追踪程序）
int flag_track_yaw = 0;
int num_count = 0;
float fly_min_z;
int Flag_reach_pad_center = 0;
int Flag_z_below_30cm = 0;
float land_max_z;


//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float satfunc(float data, float Max, float Thres);                           //限幅函数
void track_land();
void printf_land();
void generate_com(int sub_mode, float state_desired[4]);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    target_raw = *msg;

    if(target_raw.detected)
    {
        is_detected = true;
        num_count_vision_lost = 0;
        
    }else
    {
        num_count_vision_lost++;
    }

    if(num_count_vision_lost > Thres_vision_lost)
    {
        is_detected = false;
    }

    // Body frame to Inertial frame
    target_raw_inertial.frame = 1;

    target_raw_inertial.position[0] = _DroneState.position[0] + target_raw.position[0];
    target_raw_inertial.position[1] = _DroneState.position[1] + target_raw.position[1];
    target_raw_inertial.position[2] = _DroneState.position[2] + target_raw.position[2];

    target_raw_inertial.attitude[2] = _DroneState.attitude[2] + target_raw.attitude[2];
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
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
    //降落追踪控制算法 的比例参数
    nh.param<float>("landing/kpx_land", kpx_land, 0.3);
    nh.param<float>("landing/kpy_land", kpy_land, 0.3);
    nh.param<float>("landing/kpz_land", kpz_land, 0.3);

    //降落追踪控制算法的最大速度
    nh.param<float>("landing/Max_velx_land", Max_velx_land, 1.0);
    nh.param<float>("landing/Max_vely_land", Max_vely_land, 1.0);
    nh.param<float>("landing/Max_velz_land", Max_velz_land, 1.0);

    //降落追踪控制算法的速度死区
    nh.param<float>("landing/Thres_velx_land", Thres_velx_land, 0.02);
    nh.param<float>("landing/Thres_vely_land", Thres_vely_land, 0.02);
    nh.param<float>("landing/Thres_velz_land", Thres_velz_land, 0.02);

    //允许降落最大距离阈值
    nh.param<float>("landing/Thres_distance_land", Thres_distance_land, 0.2);

    //允许降落计数阈值
    nh.param<float>("landing/Thres_count_land", Thres_count_land, 30);

    //允许降落最大高度阈值
    nh.param<float>("landing/land_max_z", land_max_z, 0.3);

    //允许飞行最低高度[这个高度是指降落板上方的相对高度]
    nh.param<float>("landing/fly_min_z", fly_min_z, 0.3);

    //视觉丢失计数阈值
    nh.param<int>("landing/Thres_vision_lost", Thres_vision_lost, 30);

    //降落板高度
    nh.param<float>("landing/height_on_pad", height_on_pad, -0.4);

    UKF UKF_landing;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>模式选择<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }


    Num_StateMachine = 0;

    int flag_command;                                                  //机体系FLAG
    float state_desired[4];                                            //cin的目标位置点

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        _x_ukf = UKF_landing.Run(target_raw,0.05);

        relative_position.x = _x_ukf[0] - _DroneState.position[0];

        printf_land();

        switch (Num_StateMachine)
        {
            // input
            case 0:
                Num_StateMachine_Last = Num_StateMachine;

                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                cout << "Please input command [0 for move[ned],1 for move[body], 2 for land, 777 for autonomous landing]: "<< endl;
                cin >> flag_command;

                //777 track_land
                if (flag_command == 777)
                {
                    Num_StateMachine = 4;
                    break;
                }
                //999  land
                else if (flag_command == 2)
                {
                    Num_StateMachine = 3;
                    break;
                }

                cout << "Please input next setpoint [x y z yaw]: "<< endl;

                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> state_desired[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> state_desired[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> state_desired[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cout << "500 for input again"<< endl;
                cin >> state_desired[3];

                //500  重新输入各数值
                if (state_desired[3] == 500)
                {
                    Num_StateMachine = 0;
                }//如果是机体系移动
                else if(flag_command == 1)
                {
                    Num_StateMachine = 2;
                }//惯性系移动
                else if(flag_command == 0)
                {
                    Num_StateMachine = 1;
                }else
                {
                    Num_StateMachine = 0;
                }

            break;

            // 惯性系移动
            case 1:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = Command_Now.Move_ENU;
                generate_com(0, state_desired);
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;
                Num_StateMachine = 0;
            break;

            // 机体系移动
            case 2:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = Command_Now.Move_Body;
                generate_com(0, state_desired);
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;
                Num_StateMachine = 0;
                break;

            //Land
            case 3:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = Command_Now.Land;
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;
                break;

            //追踪降落
            case 4:

                //自主降落追踪算法
                track_land();

                //计算与降落板之间的距离
                distance_pad = sqrt(relative_position.x * relative_position.x + relative_position.y * relative_position.y);

                cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>Land State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
                //降落的2个条件：
                //1：水平距离小于阈值，即无人机抵达降落板中心位置附近
                //2：相对高度小于阈值，

                //如果 相对距离 小于 阈值，计数增加
                if(distance_pad < Thres_distance_land && is_detected == 1)
                {
                    num_count++;
                    cout<< "Distance_pad: " << distance_pad <<endl;
                    cout<< "Distance_land_count: " << num_count <<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_pad: " << distance_pad <<endl;
                    cout<< "Distance_land_count: " << num_count <<endl;
                }

                //如果计数增加超过阈值，则满足降落的第一个条件（水平距离）
                if(distance_pad < Thres_distance_land && num_count > Thres_count_land)
                {
                    cout<< "Flag_reach_pad_center: " << "true" <<endl;
                    Flag_reach_pad_center = 1;
                }else
                {
                    cout<< "Flag_reach_pad_center: " << "flase" <<endl;
                    Flag_reach_pad_center = 0;
                }

                //如果 相对高度 小于 阈值，则满足降落的第二个条件（高度条件）
                if(relative_position.z <=  land_max_z)
                {
                    cout<< "Flag_z_below_30cm: " << "true" <<endl;
                    Flag_z_below_30cm = 1;
                }else
                {
                    cout<< "Flag_z_below_30cm: " << "flase" <<endl;
                    Flag_z_below_30cm = 0;
                }

                //如果降落的两个条件都满足，则切换状态机（上锁）
                if(Flag_reach_pad_center == 1 && Flag_z_below_30cm == 1)
                {
                    // arm!
                    Num_StateMachine = 5;
                }else
                {
                    // keep track
                    Num_StateMachine = 4;
                }

                //如果视觉丢失了降落板目标，计数增加
                if(is_detected == 0)
                {
                    num_count_vision_lost++;
                    cout<< "vision_lost_num: " << num_count_vision_lost <<endl;
                }else
                {
                    num_count_vision_lost = 0;
                    cout<< "vision_lost_num: " << num_count_vision_lost <<endl;
                }

                //如果丢失计数超过阈值，则切换状态机（爬升）
                if(num_count_vision_lost > Thres_vision_lost)
                {
                    num_count_vision_lost = 0;
                    Num_StateMachine = 6;
                }

                //发布控制量
                Command_Now.header.stamp = ros::Time::now();
                command_pub.publish(Command_Now);
                Num_StateMachine_Last = Num_StateMachine;

                break;

        case 5:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = Command_Now.Idle;
            command_pub.publish(Command_Now);
            Num_StateMachine_Last = Num_StateMachine;

            break;

        case 6:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = Command_Now.Hold;
            Command_Now.Move_mode  = Command_Now.Reference_State.XY_POS_Z_VEL;  //xy pos z vel
            Command_Now.Command_ID = comid;
            Command_Now.Reference_State.position_ref[0] = 0;
            Command_Now.Reference_State.position_ref[1] = 0;
            Command_Now.Reference_State.velocity_ref[2] = 0.2;
            Command_Now.Reference_State.yaw_ref = 0;
            comid++;

            Num_StateMachine_Last = Num_StateMachine;
            command_pub.publish(Command_Now);

            cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>Search State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
            //重新获得视觉信息，计数
            if(is_detected == 1)
            {
                num_count_vision_lost++;
                cout<< "vision_regain_num: " << num_count_vision_lost <<endl;
            }else
            {
                num_count_vision_lost = 0;
                cout<< "vision_regain_num: " << num_count_vision_lost <<endl;
            }

            //如果重新获得视觉计数超过阈值，则切换状态机（追踪降落）
            if(num_count_vision_lost > Thres_vision_lost)
            {
                num_count_vision_lost = 0;
                Num_StateMachine = 4;
            }

            break;

        }

        rate.sleep();

    }

    return 0;

}


//自主降落追踪算法
void track_land()
{
    Command_Now.Mode = Command_Now.Move_Body;
    Command_Now.Command_ID = comid;
    comid++;

    //小技巧：
    //虽然视觉系统也能获得无人机和降落板的相对高度，但毕竟不是特别准确
    //因为降落板的高度已知，且在平面内运动，可通过无人机自身高度去结算与降落板的相对高度
    //relative_position.z = (height_on_pad  - pos_drone.position.z);

    //xyz速度控制模式
    Command_Now.Move_mode  = Command_Now.Reference_State.XYZ_VEL; // xy velocity z velocity
    //如果要去追踪一个动态的降落板，则需要反馈其速度
    Command_Now.Reference_State.velocity_ref[0] =  kpx_land * relative_position.x;
    Command_Now.Reference_State.velocity_ref[1] =  - kpy_land * relative_position.y;
    //z轴的期望位置是 降落板上方的某一个高度
    //之所以这么做是因为飞机飞得太低，视觉系统无法捕捉降落板
    Command_Now.Reference_State.velocity_ref[2] =  - kpz_land * (relative_position.z - fly_min_z);

    //不追踪偏航角，则锁死偏航角为0
    Command_Now.Reference_State.yaw_ref =0;

    //饱和函数
    satfunc(Command_Now.Reference_State.velocity_ref[0], Max_velx_land, Thres_velx_land);
    satfunc(Command_Now.Reference_State.velocity_ref[1], Max_vely_land, Thres_vely_land);
    satfunc(Command_Now.Reference_State.velocity_ref[2], Max_velz_land, Thres_velz_land);
}

//饱和函数
float satfunc(float data, float Max, float Thres)
{
    if (abs(data)<Thres)
    {
        return 0;
    }
    else if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}
// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4])
{
    static int comid = 1;
    Command_Now.Move_mode  = sub_mode;

//# sub_mode 2-bit value:
//# 0 for position, 1 for vel, 1st for xy, 2nd for z.
//#                   xy position     xy velocity
//# z position       	0b00(0)       0b10(2)
//# z velocity		0b01(1)       0b11(3)

    if((sub_mode & 0b10) == 0) //xy channel
    {
        Command_Now.Reference_State.position_ref[0] = state_desired[0];
        Command_Now.Reference_State.position_ref[1] = state_desired[1];
    }
    else
    {
        Command_Now.Reference_State.velocity_ref[0] = state_desired[0];
        Command_Now.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        Command_Now.Reference_State.position_ref[2] = state_desired[2];
    }
    else
    {
        Command_Now.Reference_State.velocity_ref[2] = state_desired[2];
    }


    Command_Now.Reference_State.yaw_ref = state_desired[3];
    Command_Now.Command_ID = comid;
    comid++;
}

void printf_land()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Num_StateMachine : " << Num_StateMachine <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "is_detected: " << is_detected <<endl;
    cout << "relative position: " << relative_position.x << " [m] "<< relative_position.y << " [m] "<< relative_position.z << " [m] "<<endl;
    cout << "relative_yaw: " << relative_yaw/3.1415926 *180 << " [du] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "command: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;
}
