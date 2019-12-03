/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
*
* Update Time: 2019.7.6
*
* Introduction:  PX4 Position Controller 
*         1. 从应用层节点订阅/px4_command/control_command话题（ControlCommand.msg），接收来自上层的控制指令。
*         2. 从command_from_mavros.h读取无人机的状态信息（DroneState.msg）。
*         3. 调用位置环控制算法，计算加速度控制量。可选择cascade_PID, PID, UDE, passivity-UDE, NE+UDE位置控制算法。
*         4. 通过command_to_mavros.h将计算出来的控制指令发送至飞控（通过mavros包）(mavros package will send the message to PX4 as Mavlink msg)
*         5. PX4 firmware will recieve the Mavlink msg by mavlink_receiver.cpp in mavlink module.
*         6. 发送相关信息至地面站节点(/px4_command/attitude_reference)，供监控使用。
***************************************************************************************************************************/

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <state_from_mavros.h>
#include <command_to_mavros.h>

#include <pos_controller_PID.h>
#include <pos_controller_UDE.h>
#include <pos_controller_Passivity.h>
#include <pos_controller_cascade_PID.h>
#include <pos_controller_NE.h>

#include <px4_command_utils.h>

#include <circle_trajectory.h>

#include <px4_command/ControlCommand.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/Trajectory.h>
#include <px4_command/Topic_for_log.h>
#include <px4_command/Trajectory.h>
#include <LowPassFilter.h>

#include <px4_command/ControlOutput.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
px4_command::ControlCommand Command_Now;                      //无人机当前执行命令
px4_command::ControlCommand Command_Last;                     //无人机上一条执行命令
px4_command::ControlCommand Command_to_gs;                    //发送至地面站的指令
px4_command::DroneState _DroneState;                          //无人机状态量
Eigen::Vector3d throttle_sp;
px4_command::ControlOutput _ControlOutput;
px4_command::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
float cur_time;
px4_command::Topic_for_log _Topic_for_log;                  //用于日志记录的topic

float Takeoff_height;                                       //起飞高度
float Disarm_height;                                        //自动上锁高度
float Use_accel;                                            // 1 for use the accel command
int Flag_printf;


float disturbance_a_xy,disturbance_b_xy;
float disturbance_a_z,disturbance_b_z;
float disturbance_T;
float disturbance_start_time;
float disturbance_end_time;
// For PPN landing - Silas
Eigen::Vector3d pos_des_prev;
Eigen::Vector3d vel_command;
float ppn_kx;
float ppn_ky;
float ppn_kz;//0.01;

//变量声明 - 其他变量
//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int check_failsafe();
void printf_param();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const px4_command::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
    
    // 无人机一旦接受到Land指令，则会屏蔽其他指令
    if(Command_Last.Mode == command_to_mavros::Land)
    {
        Command_Now.Mode = command_to_mavros::Land;
    }

    // Check for geo fence: If drone is out of the geo fence, it will land now.
    if(check_failsafe() == 1)
    {
        Command_Now.Mode = command_to_mavros::Land;
    }
}

void drone_state_cb(const px4_command::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    _DroneState.time_from_start = cur_time;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_controller");
    ros::NodeHandle nh("~");

    //【订阅】指令
    // 本话题来自根据需求自定义的上层模块，比如track_land.cpp 比如move.cpp
    ros::Subscriber Command_sub = nh.subscribe<px4_command::ControlCommand>("/px4_command/control_command", 10, Command_cb);

    //【订阅】无人机当前状态
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<px4_command::DroneState>("/px4_command/drone_state", 10, drone_state_cb);

    // 发布log消息至ground_station.cpp
    ros::Publisher log_pub = nh.advertise<px4_command::Topic_for_log>("/px4_command/topic_for_log", 10);

    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Use_accel", Use_accel, 0.0);
    nh.param<int>("Flag_printf", Flag_printf, 0.0);

    nh.param<float>("disturbance_a_xy", disturbance_a_xy, 0.5);
    nh.param<float>("disturbance_b_xy", disturbance_b_xy, 0.0);

    nh.param<float>("disturbance_a_z", disturbance_a_z, 0.5);
    nh.param<float>("disturbance_b_z", disturbance_b_z, 0.0);
    nh.param<float>("disturbance_T", disturbance_T, 0.0);

    nh.param<float>("disturbance_start_time", disturbance_start_time, 5.0);
    nh.param<float>("disturbance_end_time", disturbance_end_time, 10.0);

    nh.param<float>("ppn_kx", ppn_kx, 0.0);
    nh.param<float>("ppn_ky", ppn_ky, 0.0);
    nh.param<float>("ppn_kz", ppn_kz, 0.0);
    
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);

    // 位置控制一般选取为50Hz，主要取决于位置状态的更新频率
    ros::Rate rate(50.0);

    LowPassFilter LPF_x;
    LowPassFilter LPF_y;
    LowPassFilter LPF_z;

    LPF_x.set_Time_constant(disturbance_T);
    LPF_y.set_Time_constant(disturbance_T);
    LPF_z.set_Time_constant(disturbance_T);

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;
    
    // 位置控制类 - 根据switch_ude选择其中一个使用，默认为PID
    pos_controller_cascade_PID pos_controller_cascade_pid;
    pos_controller_PID pos_controller_pid;
    pos_controller_UDE pos_controller_ude;
    pos_controller_passivity pos_controller_ps;
    pos_controller_NE pos_controller_ne;

    // 选择控制律
    int switch_ude;
    cout << "Please choose the controller: 0 for cascade_PID, 1 for PID, 2 for UDE, 3 for passivity, 4 for NE: "<<endl;
    cin >> switch_ude;

    if(switch_ude == 0)
    {
        pos_controller_cascade_pid.printf_param();
    }else if(switch_ude == 1)
    {
        pos_controller_pid.printf_param();
    }else if(switch_ude == 2)
    {
        pos_controller_ude.printf_param();
    }else if(switch_ude == 3)
    {
        pos_controller_ps.printf_param();
    }else if(switch_ude == 4)
    {
        pos_controller_ne.printf_param();
    }

    // 圆形轨迹追踪类
    Circle_Trajectory _Circle_Trajectory;
    float time_trajectory = 0.0;
    _Circle_Trajectory.printf_param();

    printf_param();

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 先读取一些飞控的数据
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Set the takeoff position
    Takeoff_position[0] = _DroneState.position[0];
    Takeoff_position[1] = _DroneState.position[1];
    Takeoff_position[2] = _DroneState.position[2];

    // NE控制律需要设置起飞初始值
    if(switch_ude == 4)
    {
        pos_controller_ne.set_initial_pos(Takeoff_position);
    }

    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode = command_to_mavros::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
    Command_Now.Reference_State.position_ref[0] = 0;
    Command_Now.Reference_State.position_ref[1] = 0;
    Command_Now.Reference_State.position_ref[2] = 0;
    Command_Now.Reference_State.velocity_ref[0] = 0;
    Command_Now.Reference_State.velocity_ref[1] = 0;
    Command_Now.Reference_State.velocity_ref[2] = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref = 0;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        //执行回调函数
        ros::spinOnce();

        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case command_to_mavros::Idle:
            _command_to_mavros.idle();
            break;

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
        case command_to_mavros::Takeoff:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
            Command_to_gs.Reference_State.position_ref[0] = Takeoff_position[0];
            Command_to_gs.Reference_State.position_ref[1] = Takeoff_position[1];
            Command_to_gs.Reference_State.position_ref[2] = Takeoff_position[2] + Takeoff_height;
            Command_to_gs.Reference_State.velocity_ref[0] = 0;
            Command_to_gs.Reference_State.velocity_ref[1] = 0;
            Command_to_gs.Reference_State.velocity_ref[2] = 0;
            Command_to_gs.Reference_State.acceleration_ref[0] = 0;
            Command_to_gs.Reference_State.acceleration_ref[1] = 0;
            Command_to_gs.Reference_State.acceleration_ref[2] = 0;
            Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad

            if(switch_ude == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            
            break;

        // 【Move_ENU】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
        case command_to_mavros::Move_ENU:
            Command_to_gs = Command_Now;

            if(switch_ude == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            break;

        // 【Move_Body】 机体系移动。
        case command_to_mavros::Move_Body:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            //只有在comid增加时才会进入解算 ： 机体系 至 惯性系
            if( Command_Now.Command_ID  >  Command_Last.Command_ID )
            {
                //xy velocity mode
                if( Command_Now.Reference_State.Sub_mode  & 0b10 )
                {
                    float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
                    float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame

                    //根据无人机当前偏航角进行坐标系转换
                    px4_command_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
                    Command_to_gs.Reference_State.position_ref[0] = 0;
                    Command_to_gs.Reference_State.position_ref[1] = 0;
                    Command_to_gs.Reference_State.velocity_ref[0] = d_vel_enu[0];
                    Command_to_gs.Reference_State.velocity_ref[1] = d_vel_enu[1];
                }
                //xy position mode
                else
                {
                    float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
                    float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
                    px4_command_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);

                    Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
                    Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];
                    Command_to_gs.Reference_State.velocity_ref[0] = 0;
                    Command_to_gs.Reference_State.velocity_ref[1] = 0;
                }

                //z velocity mode
                if( Command_Now.Reference_State.Sub_mode  & 0b01 )
                {
                    Command_to_gs.Reference_State.position_ref[2] = 0;
                    Command_to_gs.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];
                }
                //z posiiton mode
                {
                    Command_to_gs.Reference_State.position_ref[2] = _DroneState.position[2] + Command_Now.Reference_State.position_ref[2];
                    Command_to_gs.Reference_State.velocity_ref[2] = 0; 
                }

                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2] + Command_Now.Reference_State.yaw_ref;

                float d_acc_body[2] = {Command_Now.Reference_State.acceleration_ref[0], Command_Now.Reference_State.acceleration_ref[1]};       
                float d_acc_enu[2]; 

                px4_command_utils::rotation_yaw(_DroneState.attitude[2], d_acc_body, d_acc_enu);
                Command_to_gs.Reference_State.acceleration_ref[0] = d_acc_enu[0];
                Command_to_gs.Reference_State.acceleration_ref[1] = d_acc_enu[1];
                Command_to_gs.Reference_State.acceleration_ref[2] = Command_Now.Reference_State.acceleration_ref[2];

            }

            if(switch_ude == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);


            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }

            break;

        // 【Hold】 悬停。当前位置悬停
        case command_to_mavros::Hold:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros::Hold)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
                Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_to_gs.Reference_State.position_ref[2] = _DroneState.position[2];
                Command_to_gs.Reference_State.velocity_ref[0] = 0;
                Command_to_gs.Reference_State.velocity_ref[1] = 0;
                Command_to_gs.Reference_State.velocity_ref[2] = 0;
                Command_to_gs.Reference_State.acceleration_ref[0] = 0;
                Command_to_gs.Reference_State.acceleration_ref[1] = 0;
                Command_to_gs.Reference_State.acceleration_ref[2] = 0;
                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            if(switch_ude == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case command_to_mavros::Land:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros::Land)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
                Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_to_gs.Reference_State.position_ref[2] = Takeoff_position[2];
                Command_to_gs.Reference_State.velocity_ref[0] = 0;
                Command_to_gs.Reference_State.velocity_ref[1] = 0;
                Command_to_gs.Reference_State.velocity_ref[2] = 0;
                Command_to_gs.Reference_State.acceleration_ref[0] = 0;
                Command_to_gs.Reference_State.acceleration_ref[1] = 0;
                Command_to_gs.Reference_State.acceleration_ref[2] = 0;
                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            //如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            if(abs(_DroneState.position[2] - Takeoff_position[2]) < Disarm_height)
            {
                if(_DroneState.mode == "OFFBOARD")
                {
                    _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                    _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
                }

                if(_DroneState.armed)
                {
                    _command_to_mavros.arm_cmd.request.value = false;
                    _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);

                }

                if (_command_to_mavros.arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {
                if(switch_ude == 0)
                {
                    _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 1)
                {
                    _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 2)
                {
                    _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 3)
                {
                    _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 4)
                {
                    _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }
                
                throttle_sp[0] = _ControlOutput.Throttle[0];
                throttle_sp[1] = _ControlOutput.Throttle[1];
                throttle_sp[2] = _ControlOutput.Throttle[2];

                _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

                if(Use_accel > 0.5)
                {
                    _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
                }else
                {
                    _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
                }
             }


            break;

        // 【Disarm】 紧急上锁。直接上锁，不建议使用，危险。
        case command_to_mavros::Disarm:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            
            if(_DroneState.mode == "OFFBOARD")
            {
                _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
            }

            if(_DroneState.armed)
            {
                _command_to_mavros.arm_cmd.request.value = false;
                _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);

            }

            if (_command_to_mavros.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        // 【PPN_land】 暂空。可进行自定义
        case command_to_mavros::PPN_land:

            if (Command_Last.Mode != command_to_mavros::PPN_land)
            {
                pos_des_prev[0] = _DroneState.position[0];
                pos_des_prev[1] = _DroneState.position[1];
                pos_des_prev[2] = _DroneState.position[2];
            }

            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;

            Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;

            vel_command[0] = ppn_kx * ( Command_Now.Reference_State.position_ref[0] - _DroneState.position[0]);
            vel_command[1] = ppn_ky * ( Command_Now.Reference_State.position_ref[1] - _DroneState.position[1]);
            vel_command[2] = ppn_kz * ( Command_Now.Reference_State.position_ref[2] - _DroneState.position[2]);

            for (int i=0; i<3; i++)
            {
                Command_to_gs.Reference_State.position_ref[i] = pos_des_prev[i] + vel_command[i]*dt;
            }

            Command_to_gs.Reference_State.velocity_ref[0] = 0;
            Command_to_gs.Reference_State.velocity_ref[1] = 0;
            Command_to_gs.Reference_State.velocity_ref[2] = 0;
            Command_to_gs.Reference_State.acceleration_ref[0] = 0;
            Command_to_gs.Reference_State.acceleration_ref[1] = 0;
            Command_to_gs.Reference_State.acceleration_ref[2] = 0;
            Command_to_gs.Reference_State.yaw_ref = 0; //rad

            for (int i=0; i<3; i++)
            {
                pos_des_prev[i] = Command_to_gs.Reference_State.position_ref[i];
            }
        
            if(switch_ude == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }

            
            break;
        
        // Trajectory_Tracking 轨迹追踪控制，与上述追踪点或者追踪速度不同，此时期望输入为一段轨迹
        case command_to_mavros::Trajectory_Tracking:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            
            if (Command_Last.Mode != command_to_mavros::Trajectory_Tracking)
            {
                time_trajectory = 0.0;
            }

            time_trajectory = time_trajectory + dt;

            Command_to_gs.Reference_State = _Circle_Trajectory.Circle_trajectory_generation(time_trajectory);

            //_Circle_Trajectory.printf_result(Command_to_gs.Reference_State);

            if(switch_ude == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }

            // 输入干扰
            Eigen::Vector3d random;

            // 先生成随机数
            random[0] = px4_command_utils::random_num(disturbance_a_xy, disturbance_b_xy);
            random[1] = px4_command_utils::random_num(disturbance_a_xy, disturbance_b_xy);
            random[2] = px4_command_utils::random_num(disturbance_a_z, disturbance_b_z);

            // 低通滤波
            random[0] = LPF_x.apply(random[0], 0.02);
            random[1] = LPF_y.apply(random[1], 0.02);
            random[2] = LPF_z.apply(random[2], 0.02);

            if(time_trajectory>disturbance_start_time && time_trajectory<disturbance_end_time)
            {
                //应用输入干扰信号
                _ControlOutput.Throttle[0] = _ControlOutput.Throttle[0] + random[0];
                _ControlOutput.Throttle[1] = _ControlOutput.Throttle[1] + random[1];
                _ControlOutput.Throttle[2] = _ControlOutput.Throttle[2] + random[2];
            }

            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            
            // Quit  悬停于最后一个目标点
            if (time_trajectory >= _Circle_Trajectory.time_total)
            {
                Command_Now.Mode = command_to_mavros::Move_ENU;
                Command_Now.Reference_State = Command_to_gs.Reference_State;
            }

            break;
        }

        if(Flag_printf == 1)
        {
            //cout <<">>>>>>>>>>>>>>>>>>>>>> px4_pos_controller <<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
            // 打印无人机状态
            px4_command_utils::prinft_drone_state(_DroneState);

            // 打印上层控制指令
            px4_command_utils::printf_command_control(Command_to_gs);

            // 打印位置控制器中间计算量
            if(switch_ude == 0)
            {
                pos_controller_cascade_pid.printf_result();
            }else if(switch_ude == 1)
            {
                pos_controller_pid.printf_result();
            }else if(switch_ude == 2)
            {
                pos_controller_ude.printf_result();
            }else if(switch_ude == 3)
            {
                pos_controller_ps.printf_result();
            }else if(switch_ude == 4)
            {
                pos_controller_ne.printf_result();
            }

            // 打印位置控制器输出结果
            px4_command_utils::prinft_attitude_reference(_AttitudeReference);

        }else if(((int)(cur_time*10) % 50) == 0)
        {
            cout << "px4_pos_controller is running for :" << cur_time << " [s] "<<endl;
        }

        // For log
        if(time_trajectory == 0)
        {
            _Topic_for_log.time = -1.0;
        }
        else
        {
            _Topic_for_log.time = time_trajectory;
        }

        _Topic_for_log.header.stamp = ros::Time::now();
        _Topic_for_log.Drone_State = _DroneState;
        _Topic_for_log.Control_Command = Command_to_gs;
        _Topic_for_log.Attitude_Reference = _AttitudeReference;
        _Topic_for_log.Control_Output = _ControlOutput;

        log_pub.publish(_Topic_for_log);

        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;

}


void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height: "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height : "<< Disarm_height <<" [m] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
    cout << "ppn_kx: "<< ppn_kx<<" [m] "<<endl;

    cout << "disturbance_a_xy: "<< disturbance_a_xy<<" [m] "<<endl;
    cout << "disturbance_b_xy: "<< disturbance_b_xy<<" [m] "<<endl;

    cout << "disturbance_a_z: "<< disturbance_a_z<<" [m] "<<endl;
    cout << "disturbance_b_z: "<< disturbance_b_z<<" [m] "<<endl;
    cout << "disturbance_T: "<< disturbance_T<<" [m] "<<endl;

    cout << "disturbance_start_time: "<< disturbance_start_time<<" [s] "<<endl;
    cout << "disturbance_end_time: "<< disturbance_end_time<<" [s] "<<endl;
    
    

}

int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        return 1;
        cout << "Out of the geo fence, the drone is landing... "<< endl;
    }
    else{
        return 0;
    }
}