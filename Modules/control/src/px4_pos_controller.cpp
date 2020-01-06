/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
*
* Update Time: 2019.12.24
*
* Introduction:  PX4 Position Controller 
*         1. 从应用层节点订阅/prometheus/control_command话题（ControlCommand.msg），接收来自上层的控制指令。
*         2. 从command_from_mavros.h读取无人机的状态信息（DroneState.msg）。
*         3. 调用位置环控制算法，计算加速度控制量。可选择cascade_PID, PID, UDE, passivity-UDE, NE+UDE位置控制算法。
*         4. 通过command_to_mavros.h将计算出来的控制指令发送至飞控（通过mavros包）(mavros package will send the message to PX4 as Mavlink msg)
*         5. PX4 firmware will recieve the Mavlink msg by mavlink_receiver.cpp in mavlink module.
*         6. 发送相关信息至地面站节点(/prometheus/attitude_reference)，供监控使用。
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

#include <prometheus_control_utils.h>
#include <circle_trajectory.h>
#include <LowPassFilter.h>

#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/TrajectoryPoint.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/Trajectory.h>
#include <prometheus_msgs/GroundStation.h>
#include <prometheus_msgs/Trajectory.h>
#include <prometheus_msgs/ControlOutput.h>


using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float cur_time;                                             //程序运行时间
int controller_number;                                      //所选择控制器编号
float Takeoff_height;                                       //默认起飞高度
float Disarm_height;                                        //自动上锁高度

//人工外界干扰
int use_disturbance;
float disturbance_a_xy,disturbance_b_xy;
float disturbance_a_z,disturbance_b_z;
float disturbance_T;
float disturbance_start_time;
float disturbance_end_time;

//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

prometheus_msgs::DroneState _DroneState;                          //无人机状态量

prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令

prometheus_msgs::ControlOutput _ControlOutput;
prometheus_msgs::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
prometheus_msgs::GroundStation _GroundStation;                   //用于发送至地面站及log的消息

Eigen::Vector3d throttle_sp;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int check_failsafe();
void printf_param();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
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

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
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
    ros::Subscriber Command_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, Command_cb);

    //【订阅】无人机当前状态
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【发布】log消息至ground_station.cpp
    ros::Publisher GS_pub = nh.advertise<prometheus_msgs::GroundStation>("/prometheus/GroundStation", 10);

    // 参数读取
    nh.param<float>("pos_controller/Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("pos_controller/Disarm_height", Disarm_height, 0.15);
    nh.param<int>("pos_controller/controller_number", controller_number, 0);

    nh.param<int>("Input_disturbance/use_disturbance", use_disturbance, 0);
    nh.param<float>("Input_disturbance/disturbance_a_xy", disturbance_a_xy, 0.0);
    nh.param<float>("Input_disturbance/disturbance_b_xy", disturbance_b_xy, 0.0);

    nh.param<float>("Input_disturbance/disturbance_a_z", disturbance_a_z, 0.0);
    nh.param<float>("Input_disturbance/disturbance_b_z", disturbance_b_z, 0.0);
    nh.param<float>("Input_disturbance/disturbance_T", disturbance_T, 0.0);

    nh.param<float>("Input_disturbance/disturbance_start_time", disturbance_start_time, 0.0);
    nh.param<float>("Input_disturbance/disturbance_end_time", disturbance_end_time, 0.0);
    
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
    
    // 位置控制类 - 根据controller_number选择其中一个使用，默认为PID
    pos_controller_cascade_PID pos_controller_cascade_pid;
    pos_controller_PID pos_controller_pid;
    pos_controller_UDE pos_controller_ude;
    pos_controller_passivity pos_controller_ps;
    pos_controller_NE pos_controller_ne;

    // 选择控制律
    if(controller_number == 0)
    {
        pos_controller_cascade_pid.printf_param();
    }else if(controller_number == 1)
    {
        pos_controller_pid.printf_param();
    }else if(controller_number == 2)
    {
        pos_controller_ude.printf_param();
    }else if(controller_number == 3)
    {
        pos_controller_ps.printf_param();
    }else if(controller_number == 4)
    {
        pos_controller_ne.printf_param();
    }

    // 圆形轨迹追踪类
    Circle_Trajectory _Circle_Trajectory;
    float time_trajectory = 0.0;
    _Circle_Trajectory.printf_param();

    printf_param();

    // 先读取一些飞控的数据
    for(int i=0;i<100;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Set the takeoff position
    Eigen::Vector3d Takeoff_position;
    Takeoff_position[0] = _DroneState.position[0];
    Takeoff_position[1] = _DroneState.position[1];
    Takeoff_position[2] = _DroneState.position[2];

    // NE控制律需要设置起飞初始值
    if(controller_number == 4)
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
    float last_time = prometheus_control_utils::get_time_in_sec(begin_time);
    float dt = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        cur_time = prometheus_control_utils::get_time_in_sec(begin_time);
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
            
            Command_Now.Reference_State.position_ref[0] = Takeoff_position[0];
            Command_Now.Reference_State.position_ref[1] = Takeoff_position[1];
            Command_Now.Reference_State.position_ref[2] = Takeoff_position[2] + Takeoff_height;
            Command_Now.Reference_State.yaw_ref = _DroneState.attitude[2];
            
            break;

        // 【Move_ENU】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
        case command_to_mavros::Move_ENU:

            break;

        // 【Move_Body】 机体系移动。
        case command_to_mavros::Move_Body:
        
            //只有在comid增加时才会进入解算 ： 机体系 至 惯性系
            if( Command_Now.Command_ID  >  Command_Last.Command_ID )
            {
                //xy velocity mode
                if( Command_Now.Reference_State.Sub_mode  & 0b10 )
                {
                    float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
                    float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame

                    //根据无人机当前偏航角进行坐标系转换
                    prometheus_control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
                    Command_Now.Reference_State.position_ref[0] = 0;
                    Command_Now.Reference_State.position_ref[1] = 0;
                    Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
                    Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
                }
                //xy position mode
                else
                {
                    float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
                    float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
                    prometheus_control_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);

                    Command_Now.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
                    Command_Now.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];
                    Command_Now.Reference_State.velocity_ref[0] = 0;
                    Command_Now.Reference_State.velocity_ref[1] = 0;
                }

                //z velocity mode
                if( Command_Now.Reference_State.Sub_mode  & 0b01 )
                {
                    Command_Now.Reference_State.position_ref[2] = 0;
                    Command_Now.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];
                }
                //z posiiton mode
                {
                    Command_Now.Reference_State.position_ref[2] = _DroneState.position[2] + Command_Now.Reference_State.position_ref[2];
                    Command_Now.Reference_State.velocity_ref[2] = 0; 
                }

                Command_Now.Reference_State.yaw_ref = _DroneState.attitude[2] + Command_Now.Reference_State.yaw_ref;

                float d_acc_body[2] = {Command_Now.Reference_State.acceleration_ref[0], Command_Now.Reference_State.acceleration_ref[1]};       
                float d_acc_enu[2]; 

                prometheus_control_utils::rotation_yaw(_DroneState.attitude[2], d_acc_body, d_acc_enu);
                Command_Now.Reference_State.acceleration_ref[0] = d_acc_enu[0];
                Command_Now.Reference_State.acceleration_ref[1] = d_acc_enu[1];
                Command_Now.Reference_State.acceleration_ref[2] = Command_Now.Reference_State.acceleration_ref[2];
            }

            break;

        // 【Hold】 悬停。当前位置悬停
        case command_to_mavros::Hold:

            if (Command_Last.Mode != command_to_mavros::Hold)
            {
                Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
                Command_Now.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_Now.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_Now.Reference_State.position_ref[2] = _DroneState.position[2];
                Command_Now.Reference_State.velocity_ref[0] = 0;
                Command_Now.Reference_State.velocity_ref[1] = 0;
                Command_Now.Reference_State.velocity_ref[2] = 0;
                Command_Now.Reference_State.acceleration_ref[0] = 0;
                Command_Now.Reference_State.acceleration_ref[1] = 0;
                Command_Now.Reference_State.acceleration_ref[2] = 0;
                Command_Now.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case command_to_mavros::Land:

            if (Command_Last.Mode != command_to_mavros::Land)
            {
                Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
                Command_Now.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_Now.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_Now.Reference_State.position_ref[2] = Takeoff_position[2];
                Command_Now.Reference_State.velocity_ref[0] = 0;
                Command_Now.Reference_State.velocity_ref[1] = 0;
                Command_Now.Reference_State.velocity_ref[2] = 0;
                Command_Now.Reference_State.acceleration_ref[0] = 0;
                Command_Now.Reference_State.acceleration_ref[1] = 0;
                Command_Now.Reference_State.acceleration_ref[2] = 0;
                Command_Now.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
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
            }

            break;
        
        // Trajectory_Tracking 轨迹追踪控制，与上述追踪点或者追踪速度不同，此时期望输入为一段轨迹
        case command_to_mavros::Trajectory_Tracking:
            
            if (Command_Last.Mode != command_to_mavros::Trajectory_Tracking)
            {
                time_trajectory = 0.0;
            }

            time_trajectory = time_trajectory + dt;

            Command_Now.Reference_State = _Circle_Trajectory.Circle_trajectory_generation(time_trajectory);

            //_Circle_Trajectory.printf_result(Command_Now.Reference_State);
            
            // Quit  
            if (time_trajectory >= _Circle_Trajectory.time_total)
            {
                Command_Now.Mode = command_to_mavros::Land;
            }

            break;

        // 【User_Mode1】 暂空。可进行自定义
        case command_to_mavros::User_Mode1:
            
            break;

        // 【User_Mode2】 暂空。可进行自定义
        case command_to_mavros::User_Mode2:
            
            break;
        }


        // 除Idle模式外,选择控制器计算控制量
        if(Command_Now.Mode != command_to_mavros::Idle)
        {
            if(controller_number == 0)
            {
                _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }else if(controller_number == 1)
            {
                _ControlOutput = pos_controller_pid.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }else if(controller_number == 2)
            {
                _ControlOutput = pos_controller_ude.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }else if(controller_number == 3)
            {
                _ControlOutput = pos_controller_ps.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }else if(controller_number == 4)
            {
                _ControlOutput = pos_controller_ne.pos_controller(_DroneState, Command_Now.Reference_State, dt);
            }
            
            if(use_disturbance == 1 && time_trajectory>disturbance_start_time && time_trajectory<disturbance_end_time) 
            {
                // 输入干扰
                Eigen::Vector3d random;

                // 先生成随机数
                random[0] = prometheus_control_utils::random_num(disturbance_a_xy, disturbance_b_xy);
                random[1] = prometheus_control_utils::random_num(disturbance_a_xy, disturbance_b_xy);
                random[2] = prometheus_control_utils::random_num(disturbance_a_z, disturbance_b_z);

                // 低通滤波
                random[0] = LPF_x.apply(random[0], 0.02);
                random[1] = LPF_y.apply(random[1], 0.02);
                random[2] = LPF_z.apply(random[2], 0.02);

                //应用输入干扰信号
                _ControlOutput.Throttle[0] = _ControlOutput.Throttle[0] + random[0];
                _ControlOutput.Throttle[1] = _ControlOutput.Throttle[1] + random[1];
                _ControlOutput.Throttle[2] = _ControlOutput.Throttle[2] + random[2];
            }
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = prometheus_control_utils::ThrottleToAttitude(throttle_sp, Command_Now.Reference_State.yaw_ref);

            _command_to_mavros.send_attitude_setpoint(_AttitudeReference); 
        }



        if(((int)(cur_time*10) % 50) == 0)
        {
            cout << "px4_pos_controller is running for :" << cur_time << " [s] "<<endl;
        }

        // For log
        if(time_trajectory == 0)
        {
            _GroundStation.time = -1.0;
        }
        else
        {
            _GroundStation.time = time_trajectory;
        }

        _GroundStation.header.stamp = ros::Time::now();
        _GroundStation.Drone_State = _DroneState;
        _GroundStation.Control_Command = Command_Now;
        _GroundStation.Attitude_Reference = _AttitudeReference;
        _GroundStation.Control_Output = _ControlOutput;

        GS_pub.publish(_GroundStation);

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