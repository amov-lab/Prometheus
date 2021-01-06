/***************************************************************************************************************************
* swarm_controller.cpp
*
* Author: Qyp
*
* Update Time: 2020.10.19
*                           采用虚拟领机-从机结构。地面站为主节点，每一个飞机都为从机                
***************************************************************************************************************************/
#include <ros/ros.h>

#include "command_to_mavros.h"
#include "prometheus_control_utils.h"
#include "swarm_control_utils.h"
#include "message_utils.h"

#define NODE_NAME "swarm_controller"

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int swarm_num;
string uav_name;
int uav_id,neighbour_id1,neighbour_id2;
string neighbour_name1,neighbour_name2;
int num_neighbour = 2;
float cur_time;                                             //程序运行时间
float Takeoff_height;                                       //默认起飞高度
float Disarm_height;                                        //自动上锁高度
float Land_speed;                                           //降落速度
Eigen::MatrixXf formation_separation;

// 速度控制参数
float k_p;
float k_aij;
float k_gamma;

bool flag_printf;

//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position;                              // 起飞位置
prometheus_msgs::DroneState _DroneState;                         //无人机状态量

Eigen::Vector3d pos_drone;
Eigen::Vector3d vel_drone;

Eigen::Vector3d pos_nei[2];
Eigen::Vector3d vel_nei[2];


prometheus_msgs::SwarmCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::SwarmCommand Command_Last;                     //无人机上一条执行命令

Eigen::Vector3d state_sp(0,0,0);
Eigen::Vector3d state_sp_extra(0,0,0);
float yaw_sp;

prometheus_msgs::Message message;
prometheus_msgs::LogMessage LogMessage;

//RVIZ显示：期望位置
geometry_msgs::PoseStamped ref_pose_rviz;
float dt = 0;

ros::Publisher rivz_ref_pose_pub;
ros::Publisher message_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();
int check_failsafe();
void printf_state();
geometry_msgs::PoseStamped get_rviz_ref_posistion(const prometheus_msgs::SwarmCommand& cmd);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    // CommandID必须递增才会被记录
    Command_Now = *msg;
    
    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == prometheus_msgs::SwarmCommand::Disarm)
    {
        Command_Now = Command_Last;
    }

    if(Command_Now.Mode == prometheus_msgs::SwarmCommand::Position_Control ||
        Command_Now.Mode == prometheus_msgs::SwarmCommand::Velocity_Control ||
        Command_Now.Mode == prometheus_msgs::SwarmCommand::Accel_Control )
    {
        formation_separation = swarm_control_utils::get_formation_separation(Command_Now.swarm_shape, Command_Now.swarm_size, swarm_num);
    }
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    pos_drone  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_drone  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}

void nei_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int nei_id)
{
    pos_nei[nei_id]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_nei[nei_id]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}

void timerCallback(const ros::TimerEvent& e)
{
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "[" + uav_name + "] : Program is running.");
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_controller");
    ros::NodeHandle nh("~");
    // 建议控制频率 ： 10 - 50Hz, 控制频率取决于控制形式，若控制方式为速度或加速度应适当提高频率
    ros::Rate rate(20.0);

    //无人机编号 1号无人机则为1
    nh.param<int>("swarm_num", swarm_num, 1);
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("uav_name", uav_name, "/uav0");
    nh.param<float>("k_p", k_p, 0.95);
    nh.param<float>("k_aij", k_aij, 0.1);
    nh.param<float>("k_gamma", k_gamma, 0.1);
    //可监听到的无人机编号，目前设定为可监听到两台无人机，后期考虑可通过数组传递参数，监听任意ID的无人机
    nh.param<int>("neighbour_id1", neighbour_id1, 0);
    nh.param<int>("neighbour_id2", neighbour_id2, 0);
    nh.param<string>("neighbour_name1", neighbour_name1, "/uav0");
    nh.param<string>("neighbour_name2", neighbour_name2, "/uav0");

    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Land_speed", Land_speed, 0.2);

    nh.param<bool>("flag_printf", flag_printf, true);

    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);

    formation_separation = Eigen::MatrixXf::Zero(swarm_num,4); 

    //【订阅】集群控制指令
    ros::Subscriber command_sub = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10, swarm_command_cb);


    //【订阅】本机状态信息
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】邻居飞机的状态信息
    ros::Subscriber nei1_state_sub = nh.subscribe<prometheus_msgs::DroneState>(neighbour_name1 + "/prometheus/drone_state", 10, boost::bind(&nei_state_cb,_1, 0));
    ros::Subscriber nei2_state_sub = nh.subscribe<prometheus_msgs::DroneState>(neighbour_name2 + "/prometheus/drone_state", 10, boost::bind(&nei_state_cb,_1, 1));

    //【发布】参考位姿 RVIZ显示用
    rivz_ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/prometheus/control/ref_pose_rviz", 10);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 10秒定时打印，以确保程序在正确运行
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timerCallback);

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;

    if(flag_printf)
    {
        printf_param();
    }
    
    
    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode                                = prometheus_msgs::SwarmCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.position_ref[0]     = 0;
    Command_Now.position_ref[1]     = 0;
    Command_Now.position_ref[2]     = 0;
    Command_Now.yaw_ref             = 0;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = prometheus_control_utils::get_time_in_sec(begin_time);
    float yita;
    Eigen::Vector3d accel_sp;
     Eigen::Vector3d throttle_sp;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        static int exec_num=0;
        exec_num++;
        // 当前时间
        cur_time = prometheus_control_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.02, 0.1);
        last_time = cur_time;

        // 执行回调函数
        ros::spinOnce();

        if(flag_printf)
        {
            printf_state();
        }

        // Check for geo fence: If drone is out of the geo fence, it will land now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = prometheus_msgs::SwarmCommand::Land;
        }

        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case prometheus_msgs::SwarmCommand::Idle:
            
            _command_to_mavros.idle();

            // 设定yaw_ref=999时，切换offboard模式，并解锁
            if(Command_Now.yaw_ref == 999)
            {
                if(_DroneState.mode != "OFFBOARD")
                {
                    _command_to_mavros.mode_cmd.request.custom_mode = "OFFBOARD";
                    _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Setting to OFFBOARD Mode...");
                }

                if(!_DroneState.armed)
                {
                    _command_to_mavros.arm_cmd.request.value = true;
                    _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Arming...");
                }
            }
            break;

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度    
        case prometheus_msgs::SwarmCommand::Takeoff:
            
            // 设定起飞点
            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Takeoff)
            {
                // 设定起飞位置
                Takeoff_position[0] = _DroneState.position[0];
                Takeoff_position[1] = _DroneState.position[1];
                Takeoff_position[2] = _DroneState.position[2];

                //
                Command_Now.position_ref[0] = Takeoff_position[0];
                Command_Now.position_ref[1] = Takeoff_position[1];
                Command_Now.position_ref[2] = Takeoff_position[2] + Takeoff_height;
                Command_Now.yaw_ref         = _DroneState.attitude[2];
            
                state_sp = Eigen::Vector3d(Takeoff_position[0],Takeoff_position[1],Takeoff_position[2] + Takeoff_height);
            }
            _command_to_mavros.send_pos_setpoint(state_sp, Command_Now.yaw_ref);
                
            break;

        // 【Hold】 悬停。当前位置悬停
        case prometheus_msgs::SwarmCommand::Hold:

            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Hold)
            {
                Command_Now.position_ref[0] = _DroneState.position[0];
                Command_Now.position_ref[1] = _DroneState.position[1];
                Command_Now.position_ref[2] = _DroneState.position[2];
                Command_Now.yaw_ref         = _DroneState.attitude[2]; //rad

                state_sp = Eigen::Vector3d(_DroneState.position[0],_DroneState.position[1],_DroneState.position[2]);
            }
            _command_to_mavros.send_pos_setpoint(state_sp, Command_Now.yaw_ref);

            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case prometheus_msgs::SwarmCommand::Land:
            if (Command_Last.Mode != prometheus_msgs::ControlCommand::Land)
            {
                Command_Now.position_ref[0] = _DroneState.position[0];
                Command_Now.position_ref[1] = _DroneState.position[1];
                Command_Now.yaw_ref         = _DroneState.attitude[2]; //rad
            }

            if(_DroneState.position[2] > Disarm_height)
            {
                Command_Now.position_ref[2] = _DroneState.position[2] - Land_speed * dt ;
                Command_Now.velocity_ref[0] = 0.0;
                Command_Now.velocity_ref[1] =  0.0;
                Command_Now.velocity_ref[2] = - Land_speed; //Land_speed

                state_sp = Eigen::Vector3d(Command_Now.position_ref[0],Command_Now.position_ref[1], Command_Now.position_ref[2] );
                state_sp_extra = Eigen::Vector3d(0.0, 0.0 , Command_Now.velocity_ref[2]);
                yaw_sp = Command_Now.yaw_ref;
                 _command_to_mavros.send_pos_vel_xyz_setpoint(state_sp, state_sp_extra, yaw_sp);
            }else
            {
                //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,直接使用飞控中的land模式
                _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);

                _command_to_mavros.arm_cmd.request.value = false;
                _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Disarming...");

                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "LAND: switch to MANUAL filght mode");
            }

            break;

        case prometheus_msgs::SwarmCommand::Disarm:

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Disarm: switch to MANUAL flight mode");
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
            break;

        case prometheus_msgs::SwarmCommand::Position_Control:

            //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点
            state_sp[0] = Command_Now.position_ref[0] + formation_separation(uav_id-1,0);
            state_sp[1] = Command_Now.position_ref[1] + formation_separation(uav_id-1,1);
            state_sp[2] = Command_Now.position_ref[2] + formation_separation(uav_id-1,2);
            yaw_sp = Command_Now.yaw_ref + formation_separation(uav_id-1,3);
            _command_to_mavros.send_pos_setpoint(state_sp, yaw_sp);

            break;

        case prometheus_msgs::SwarmCommand::Velocity_Control:

            //　平面阵型，xy控制速度，z轴高度定高
            //　一阶积分器分布式编队控制算法，可追踪时变轨迹，此处采用双向环的拓扑结构，且仅部分无人机可收到地面站发来的虚拟领队消息
            //　参考文献：Multi-Vehicle consensus with a time-varying reference state 公式(11) - (12)
            //　目前该算法有一定控制偏差，可能是由于参数选取不是最佳导致的
           
            yita = 1/ ((float)swarm_num * k_aij + k_p);

            state_sp[0] = - yita * k_aij * ( vel_nei[0][0] - k_gamma *((pos_drone[0] - pos_nei[0][0]) - ( formation_separation(uav_id-1,0) -  formation_separation(neighbour_id1-1,0)))) 
                            - yita * k_aij * ( vel_nei[1][0] - k_gamma *((pos_drone[0] - pos_nei[1][0]) - ( formation_separation(uav_id-1,0) -  formation_separation(neighbour_id2-1,0))))
                            + yita * k_p * ( Command_Now.velocity_ref[0] - k_gamma * (pos_drone[0] - Command_Now.position_ref[0] - formation_separation(uav_id-1,0)));
            state_sp[1] = - yita * k_aij * ( vel_nei[0][1] - k_gamma *((pos_drone[1] - pos_nei[0][1]) - ( formation_separation(uav_id-1,1) -  formation_separation(neighbour_id1-1,1)))) 
                            - yita * k_aij * ( vel_nei[1][1] - k_gamma *((pos_drone[1] - pos_nei[1][1]) - ( formation_separation(uav_id-1,1) -  formation_separation(neighbour_id2-1,1))))
                            + yita * k_p * ( Command_Now.velocity_ref[1] - k_gamma * (pos_drone[1] - Command_Now.position_ref[1] - formation_separation(uav_id-1,1)));
            state_sp[2] = Command_Now.position_ref[2] + formation_separation(uav_id-1,2);
            yaw_sp = Command_Now.yaw_ref + formation_separation(uav_id-1,3);

            _command_to_mavros.send_vel_xy_pos_z_setpoint(state_sp, yaw_sp);

            break;

        case prometheus_msgs::SwarmCommand::Accel_Control:

            //To be continued; 此处加速度控制实则控制的是３轴油门，限制幅度为[-1, 1] , [-1, 1], [0,1]

            //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点,并计算期望加速度（期望油门）
            //　此处也可以根据自己的算法改为　分布式控制
            //　需要增加积分项　否则会有静差

            accel_sp[0] =  2.5 * (Command_Now.position_ref[0] + formation_separation(uav_id-1,0) - pos_drone[0]) + 3.0 * (Command_Now.velocity_ref[0] - vel_drone[0]);
            accel_sp[1] =  2.5 * (Command_Now.position_ref[1] + formation_separation(uav_id-1,1) - pos_drone[1]) + 3.0 * (Command_Now.velocity_ref[1] - vel_drone[1]);
            accel_sp[2] =  2.0 * (Command_Now.position_ref[2] + formation_separation(uav_id-1,2) - pos_drone[2]) + 3.0 * (Command_Now.velocity_ref[2] - vel_drone[2]) + 9.8;
            
            //　从加速度归一化到油门
            throttle_sp =  swarm_control_utils::accelToThrottle(accel_sp, 1.0, 20.0);

            state_sp[0] = throttle_sp[0] ;
            state_sp[1] = throttle_sp[1] ;
            state_sp[2] = throttle_sp[2] ;

            yaw_sp = Command_Now.yaw_ref + formation_separation(uav_id-1,3);
            _command_to_mavros.send_acc_xyz_setpoint(state_sp, yaw_sp);

            break;

        case prometheus_msgs::SwarmCommand::Swarm_Planner:

            //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点
            state_sp[0] = Command_Now.position_ref[0];
            state_sp[1] = Command_Now.position_ref[1];
            state_sp[2] = Command_Now.position_ref[2];
            yaw_sp = Command_Now.yaw_ref;
            _command_to_mavros.send_pos_setpoint(state_sp, yaw_sp);

            break;

        case prometheus_msgs::SwarmCommand::User_Mode1:

            state_sp = Eigen::Vector3d(Command_Now.position_ref[0],Command_Now.position_ref[1],Command_Now.position_ref[2]);
            state_sp_extra = Eigen::Vector3d(Command_Now.velocity_ref[0], Command_Now.velocity_ref[1] ,Command_Now.velocity_ref[2]);
            yaw_sp = Command_Now.yaw_ref;
            _command_to_mavros.send_pos_vel_xyz_setpoint(state_sp, state_sp_extra,yaw_sp);

            break;
        }

        //发布用于RVIZ显示的位姿
        ref_pose_rviz = get_rviz_ref_posistion(Command_Now);   
        rivz_ref_pose_pub.publish(ref_pose_rviz);

        //发布log消息，可用rosbag记录
        // LogMessage.time = cur_time;
        // LogMessage.Drone_State = _DroneState;
        // LogMessage.Control_Command = Command_Now;

        // log_message_pub.publish(LogMessage);

        Command_Last = Command_Now;
        rate.sleep();
    }

}


void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> swarm controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "uav_name   : "<< uav_name <<endl;
    cout << "neighbour_name1   : "<< neighbour_name1 <<endl;
    cout << "neighbour_name2   : "<< neighbour_name2 <<endl;
    cout << "k_p    : "<< k_p <<"  "<<endl;
    cout << "k_aij       : "<< k_aij <<"  "<<endl;
    cout << "k_gamma       : "<< k_gamma <<"  "<<endl;
    

    cout << "Takeoff_height   : "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height    : "<< Disarm_height <<" [m] "<<endl;
    cout << "Land_speed       : "<< Land_speed <<" [m/s] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
}


void printf_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Swarm Controller  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << "UAV_id : " <<  uav_id << "   UAV_name : " <<  uav_name << endl;
    cout << "neighbour_id1 : " <<  neighbour_id1 << "   neighbour_name1 : " <<  neighbour_name1 << endl;
    cout << "neighbour_id2 : " <<  neighbour_id2 << "   neighbour_name2 : " <<  neighbour_name2 << endl;
    cout << "UAV_pos [X Y Z] : " << pos_drone[0] << " [ m ] "<< pos_drone[1]<<" [ m ] "<<pos_drone[2]<<" [ m ] "<<endl;
    cout << "UAV_vel [X Y Z] : " << vel_drone[0] << " [ m/s ] "<< vel_drone[1]<<" [ m/s ] "<<vel_drone[2]<<" [ m/s ] "<<endl;
    cout << "neighbour_pos [X Y Z] : " << pos_nei[0][0] << " [ m ] "<< pos_nei[0][1]<<" [ m ] "<<pos_nei[0][2]<<" [ m ] "<<endl;
    cout << "neighbour_vel [X Y Z] : " << vel_nei[0][0] << " [ m/s ] "<< vel_nei[0][1]<<" [ m/s ] "<<vel_nei[0][2]<<" [ m/s ] "<<endl;
    cout << "neighbour_pos [X Y Z] : " << pos_nei[1][0] << " [ m ] "<< pos_nei[1][1]<<" [ m ] "<<pos_nei[1][2]<<" [ m ] "<<endl;
    cout << "neighbour_vel [X Y Z] : " << vel_nei[1][0] << " [ m/s ] "<< vel_nei[1][1]<<" [ m/s ] "<<vel_nei[1][2]<<" [ m/s ] "<<endl;
}


int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        pub_message(message_pub, prometheus_msgs::Message::ERROR, NODE_NAME, "Out of the geo fence, the drone is landing...");
        return 1;
    }
    else{
        return 0;
    }
}

geometry_msgs::PoseStamped get_rviz_ref_posistion(const prometheus_msgs::SwarmCommand& cmd)
{
    geometry_msgs::PoseStamped ref_pose;

    ref_pose.header.stamp = ros::Time::now();
    // world: 世界系,即gazebo坐标系,参见tf_transform.launch
    ref_pose.header.frame_id = "world";

    if(cmd.Mode == prometheus_msgs::SwarmCommand::Idle)
    {
        ref_pose.pose.position.x = _DroneState.position[0];
        ref_pose.pose.position.y = _DroneState.position[1];
        ref_pose.pose.position.z = _DroneState.position[2];
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }else if(cmd.Mode == prometheus_msgs::SwarmCommand::Takeoff || cmd.Mode == prometheus_msgs::SwarmCommand::Hold)
    {
        ref_pose.pose.position.x = cmd.position_ref[0];
        ref_pose.pose.position.y = cmd.position_ref[1];
        ref_pose.pose.position.z = cmd.position_ref[2];
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }else if(cmd.Mode == prometheus_msgs::SwarmCommand::Disarm  || cmd.Mode == prometheus_msgs::SwarmCommand::Land )
    {
        ref_pose.pose.position.x = cmd.position_ref[0];
        ref_pose.pose.position.y = cmd.position_ref[1];
        ref_pose.pose.position.z = 0.0;
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }
    else if(cmd.Mode == prometheus_msgs::SwarmCommand::Position_Control)
    {
        ref_pose.pose.position.x = cmd.position_ref[0] + formation_separation(uav_id-1,0);
        ref_pose.pose.position.y = cmd.position_ref[1] + formation_separation(uav_id-1,1);
        ref_pose.pose.position.z = cmd.position_ref[2] + formation_separation(uav_id-1,2);
    }else if(cmd.Mode == prometheus_msgs::SwarmCommand::Velocity_Control)
    {
        ref_pose.pose.position.x = cmd.position_ref[0] + formation_separation(uav_id-1,0);
        ref_pose.pose.position.y = cmd.position_ref[1] + formation_separation(uav_id-1,1);
        ref_pose.pose.position.z = cmd.position_ref[2] + formation_separation(uav_id-1,2);
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }else if(cmd.Mode == prometheus_msgs::SwarmCommand::Accel_Control)
    {       
        ref_pose.pose.position.x = cmd.position_ref[0] + formation_separation(uav_id-1,0);
        ref_pose.pose.position.y = cmd.position_ref[1] + formation_separation(uav_id-1,1);
        ref_pose.pose.position.z = cmd.position_ref[2] + formation_separation(uav_id-1,2);
    }else
    {
        ref_pose.pose.position.x = 0.0;
        ref_pose.pose.position.y = 0.0;
        ref_pose.pose.position.z = 0.0;
        ref_pose.pose.orientation = _DroneState.attitude_q;
    }

    return ref_pose;
}
