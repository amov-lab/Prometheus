/***************************************************************************************************************************
 * target_tracking.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.4.17
 *
 * 说明: 目标追踪示例程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及追踪策略
 *      3. 发布上层控制指令 (px4_command::ControlCommand)
***************************************************************************************************************************/
//ros头文件
#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/ControlCommand.h>
#include <command_to_mavros.h>
#include <geometry_msgs/Pose.h>


using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Vision---------------------------------------------
geometry_msgs::Pose pos_target;                                 //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

int flag_detected = 0;                                          // 是否检测到目标标志
//---------------------------------------Track---------------------------------------------
int Num_StateMachine = 0;                                       // 状态机编号
int Num_StateMachine_Last = 0;                                  // 状态机编号last

float delta_x;
float distance_thres;
float kpx_track;                                                //追踪比例系数
float kpy_track;                                                //追踪比例系数
float kpz_track;                                                //追踪比例系数

int flag_x;                                                     //前后是否追踪flag

float track_max_vel_x;                                          //追踪最大速度
float track_max_vel_y;                                          //追踪最大速度
float track_max_vel_z;                                          //追踪最大速度

float track_thres_vel_x;                                          //追踪速度死区
float track_thres_vel_y;                                          //追踪速度死区
float track_thres_vel_z;                                          //追踪速度死区

int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int count_vision_lost = 0;                                                          //视觉丢失计数器阈值
//---------------------------------------Output---------------------------------------------
px4_command::ControlCommand Command_Now;                               //发送给position_control.cpp的命令

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();                                                                 //打印函数
void generate_com(int sub_mode, float state_desired[4]);
float satfunc(float data, float Max, float Thres);                                   //限幅函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    pos_target = *msg;

    if(pos_target.orientation.w == 0)
    {
        num_count_vision_lost++;
    }else if(pos_target.orientation.w == 1)
    {
        flag_detected = 1;
        num_count_vision_lost = 0;
    }

    if(num_count_vision_lost > count_vision_lost)
    {
        flag_detected = 0;
    }

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
    ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::Pose>("/vision/target", 10, vision_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::ControlCommand>("/px4_command/control_command", 10);

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(20.0);

    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪算法比例参数
    nh.param<float>("kpx_track", kpx_track, 1.0);
    nh.param<float>("kpy_track", kpy_track, 1.0);
    nh.param<float>("kpz_track", kpz_track, 1.0);

    //追踪算法最大追踪速度
    //取决与使用场景，室内或者第一次实验时，建议选小一点
    nh.param<float>("track_max_vel_x", track_max_vel_x, 0.5);
    nh.param<float>("track_max_vel_y", track_max_vel_y, 0.5);
    nh.param<float>("track_max_vel_z", track_max_vel_z, 0.5);

    //追踪算法速度死区
    nh.param<float>("track_thres_vel_x", track_thres_vel_x, 0.02);
    nh.param<float>("track_thres_vel_y", track_thres_vel_y, 0.02);
    nh.param<float>("track_thres_vel_z", track_thres_vel_z, 0.02);

    //前后方向是否追踪标志位 1 for track x, 0 for not track x
    //设计这个标志位是为了刚开始测试的时候不作前后的位移，较为安全
    nh.param<int>("flag_x", flag_x, 0);

    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("count_vision_lost", count_vision_lost, 20);

    //追踪的前后间隔
    nh.param<float>("delta_x", delta_x, 1.5);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);


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

    int flag_command;                                                  //机体系FLAG
    float state_desired[4];                                            //cin的目标位置点

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        printf_result();

        switch (Num_StateMachine)
        {
            // input
            case 0:
                Num_StateMachine_Last = Num_StateMachine;

                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;

                cout << "Please input command [0 for move[ned],1 for move[body], 2 for land, 777 for track]: "<< endl;
                cin >> flag_command;

                //777 track
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
                cout << "500 for input again: "<< endl;
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
                Command_Now.Mode = command_to_mavros::Move_ENU;
                generate_com(0, state_desired);
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;
                Num_StateMachine = 0;
            break;

            // 机体系移动
            case 2:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = command_to_mavros::Move_Body;
                generate_com(0, state_desired);
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;
                Num_StateMachine = 0;
            break;

            //Land
            case 3:
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode = command_to_mavros::Land;
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;
            break;

            //Track
            case 4:
                //首先计算距离期望位置距离
                float distance;

                if (flag_x == 0)
                {
                    distance = sqrt( pos_target.position.y * pos_target.position.y);
                }else
                {
                    distance = sqrt((pos_target.position.x-delta_x) * (pos_target.position.x-delta_x) + pos_target.position.y * pos_target.position.y);
                }

                cout << "distance : "<< distance << endl;

                //如果 视觉丢失目标 或者 当前与目标距离小于距离阈值
                //发送悬停指令
                if(flag_detected == 0 || (distance < distance_thres))
                {
                    Command_Now.Mode = command_to_mavros::Hold;
                }
                //如果捕捉到目标
                else
                {
                    //追踪是在机体系下完成
                    Command_Now.Mode = command_to_mavros::Move_Body;
                    Command_Now.Reference_State.Sub_mode  = command_to_mavros::XY_VEL_Z_POS;   //xy velocity z position
                    Command_Now.Command_ID = comid;
                    comid++;

                    if (flag_x == 0)
                    {
                        Command_Now.Reference_State.velocity_ref[0] =  0;
                    }else
                    {
                        Command_Now.Reference_State.velocity_ref[0] =  kpx_track * (pos_target.position.x - delta_x);
                    }

                    Command_Now.Reference_State.velocity_ref[1] =  - kpy_track * pos_target.position.y;

                    //Height is locked.
                    //Command_Now.Reference_State.velocity_ref[2] =  - kpz_track * pos_target.position.z;
                    Command_Now.Reference_State.position_ref[2] =  0;

                    //目前航向角锁定
                    Command_Now.Reference_State.yaw_ref = 0;

                    //速度限幅
                    Command_Now.Reference_State.velocity_ref[0] = satfunc(Command_Now.Reference_State.velocity_ref[0], track_max_vel_x, track_thres_vel_x);
                    Command_Now.Reference_State.velocity_ref[1] = satfunc(Command_Now.Reference_State.velocity_ref[1], track_max_vel_y, track_thres_vel_y);
                   // Command_Now.Reference_State.velocity_ref[2] = satfunc(Command_Now.Reference_State.velocity_ref[2], track_max_vel_z, track_thres_vel_z);

                    //如果期望速度为0,则直接执行悬停指令
                    if(Command_Now.Reference_State.velocity_ref[0]==0 && Command_Now.Reference_State.velocity_ref[1] == 0)
                    {
                        Command_Now.Mode = command_to_mavros::Hold;
                    }

                }

                //Publish
                Command_Now.header.stamp = ros::Time::now();
                command_pub.publish(Command_Now);

                Num_StateMachine_Last = Num_StateMachine;


            break;
         }
        rate.sleep();
    }

    return 0;

}

void printf_result()
{
    cout.setf(ios::fixed);
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Num_StateMachine : " << Num_StateMachine <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "flag_detected: " <<  flag_detected <<endl;
    cout << "num_count_vision_lost: " <<  num_count_vision_lost <<endl;

    cout << "pos_target: [X Y Z] : " << " " << pos_target.position.x  << " [m] "<< pos_target.position.y  <<" [m] "<< pos_target.position.z <<" [m] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Command: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "delta_x : "<< delta_x << endl;

    cout << "kpx_track : "<< kpx_track << endl;
    cout << "kpy_track : "<< kpy_track << endl;
    cout << "kpz_track : "<< kpz_track << endl;

    cout << "track_max_vel_x : "<< track_max_vel_x << endl;
    cout << "track_max_vel_y : "<< track_max_vel_y << endl;
    cout << "track_max_vel_z : "<< track_max_vel_z << endl;


    cout << "track_thres_vel_x : "<< track_thres_vel_x << endl;
    cout << "track_thres_vel_y : "<< track_thres_vel_y << endl;
    cout << "track_thres_vel_z : "<< track_thres_vel_z << endl;
    cout << "flag_x : "<< flag_x << endl;
    cout << "count_vision_lost : "<< count_vision_lost << endl;



}

// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4])
{
    static int comid = 1;
    Command_Now.Reference_State.Sub_mode  = sub_mode;

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


    Command_Now.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
    Command_Now.Command_ID = comid;
    comid++;
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


