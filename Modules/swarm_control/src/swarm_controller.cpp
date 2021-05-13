#include "swarm_controller.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    // 建议控制频率 ： 10 - 50Hz, 控制频率取决于控制形式，若控制方式为速度或加速度应适当提高频率
    ros::Rate rate(20.0);

    // 集群数量
    nh.param<int>("swarm_num", swarm_num, 1);
    // 无人机编号 1号无人机则为1
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("uav_name", uav_name, "/uav0");
    //可监听到的无人机编号，目前设定为可监听到两台无人机，后期考虑可通过数组传递参数，监听任意ID的无人机
    nh.param<int>("neighbour_id1", neighbour_id1, 0);
    nh.param<int>("neighbour_id2", neighbour_id2, 0);
    nh.param<string>("neighbour_name1", neighbour_name1, "/uav0");
    nh.param<string>("neighbour_name2", neighbour_name2, "/uav0");
    // 控制变量
    nh.param<float>("k_p", k_p, 0.95);
    nh.param<float>("k_aij", k_aij, 0.1);
    nh.param<float>("k_gamma", k_gamma, 0.1);
    // 起飞高度,上锁高度,降落速度
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Land_speed", Land_speed, 0.2);
    // 是否打印消息
    nh.param<bool>("flag_printf", flag_printf, false);
    // 地理围栏
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);
    // 如果是使用的ekf2_gps则需要设置，　如果使用的是ekf2_vision则不需要
    nh.param<float>("gazebo_offset_x", gazebo_offset[0], 0);
    nh.param<float>("gazebo_offset_y", gazebo_offset[1], 0);
    nh.param<float>("gazebo_offset_z", gazebo_offset[2], 0);

    msg_name = uav_name + "/control";

    //【订阅】集群控制指令
    command_sub = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10, swarm_command_cb);

    //【订阅】本机状态信息
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】邻居飞机的状态信息
    if(neighbour_name1 != "/uav0" || neighbour_name2 != "/uav0")
    {
        nei1_state_sub = nh.subscribe<prometheus_msgs::DroneState>(neighbour_name1 + "/prometheus/drone_state", 10, boost::bind(&nei_state_cb,_1, 0));
        nei2_state_sub = nh.subscribe<prometheus_msgs::DroneState>(neighbour_name2 + "/prometheus/drone_state", 10, boost::bind(&nei_state_cb,_1, 1));
    }

    // 【发布】位置/速度/加速度期望值 坐标系 ENU系
    //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 【服务】解锁/上锁
    //  本服务通过Mavros功能包 /plugins/command.cpp 实现
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");

    // 【服务】修改系统模式
    //  本服务通过Mavros功能包 /plugins/command.cpp 实现
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");

    if(flag_printf)
    {
        printf_param();
    }
    
    init();

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 执行回调函数
        ros::spinOnce();

        // 一般选择不打印
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
            
            idle();

            // 设定yaw_ref=999时，切换offboard模式，并解锁
            if(Command_Now.yaw_ref == 999)
            {
                if(_DroneState.mode != "OFFBOARD")
                {
                    mode_cmd.request.custom_mode = "OFFBOARD";
                    set_mode_client.call(mode_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Setting to OFFBOARD Mode...");
                }

                if(!_DroneState.armed)
                {
                    arm_cmd.request.value = true;
                    arming_client.call(arm_cmd);
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Arming...");
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
                state_sp = Eigen::Vector3d(Takeoff_position[0],Takeoff_position[1],Takeoff_position[2] + Takeoff_height);
                yaw_sp              = _DroneState.attitude[2]; 
            }
            send_pos_setpoint(state_sp, yaw_sp);
            
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
            send_pos_setpoint(state_sp, Command_Now.yaw_ref);

            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case prometheus_msgs::SwarmCommand::Land:
            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Land)
            {
                Command_Now.position_ref[0] = _DroneState.position[0];
                Command_Now.position_ref[1] = _DroneState.position[1];
                Command_Now.yaw_ref         = _DroneState.attitude[2]; //rad
            }

            if(_DroneState.position[2] > Disarm_height)
            {
                Command_Now.position_ref[2] = _DroneState.position[2] - Land_speed * 0.02 ;
                Command_Now.velocity_ref[0] = 0.0;
                Command_Now.velocity_ref[1] =  0.0;
                Command_Now.velocity_ref[2] = - Land_speed; //Land_speed

                state_sp = Eigen::Vector3d(Command_Now.position_ref[0],Command_Now.position_ref[1], Command_Now.position_ref[2] );
                state_sp_extra = Eigen::Vector3d(0.0, 0.0 , Command_Now.velocity_ref[2]);
                yaw_sp = Command_Now.yaw_ref;
                send_pos_vel_xyz_setpoint(state_sp, state_sp_extra, yaw_sp);
            }else
            {
                //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,直接使用飞控中的land模式
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);

                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Disarming...");
                pub_message(message_pub, prometheus_msgs::Message::WARN, msg_name, "LAND: switch to MANUAL filght mode");
            }

            break;

        case prometheus_msgs::SwarmCommand::Disarm:

            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_name, "Disarm: switch to MANUAL flight mode");
            if(_DroneState.mode == "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);
            }

            if(_DroneState.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }
            break;

        case prometheus_msgs::SwarmCommand::Position_Control:

            //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点
            state_sp[0] = Command_Now.position_ref[0] + formation_separation(uav_id-1,0) - gazebo_offset[0];
            state_sp[1] = Command_Now.position_ref[1] + formation_separation(uav_id-1,1) - gazebo_offset[1];
            state_sp[2] = Command_Now.position_ref[2] + formation_separation(uav_id-1,2) - gazebo_offset[2];
            yaw_sp = Command_Now.yaw_ref;
            send_pos_setpoint(state_sp, yaw_sp);
            // cout << "curPos" << Command_Now.position_ref[0] << " " << Command_Now.position_ref[1] << " " << Command_Now.position_ref[2] << endl;
            // cout << "2pos:" << state_sp[0] << " " << state_sp[1] << " " << state_sp[2] << endl;
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

            send_vel_xy_pos_z_setpoint(state_sp, yaw_sp);

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
            throttle_sp =  accelToThrottle(accel_sp, 1.0, 20.0);

            state_sp[0] = throttle_sp[0] ;
            state_sp[1] = throttle_sp[1] ;
            state_sp[2] = throttle_sp[2] ;

            yaw_sp = Command_Now.yaw_ref + formation_separation(uav_id-1,3);
            send_acc_xyz_setpoint(state_sp, yaw_sp);

            break;

        // 单个飞机情况
        case prometheus_msgs::SwarmCommand::Move:

            //　此控制方式即为　期望位置点控制, 仅针对单个飞机
            if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS)
            {
                state_sp[0] = Command_Now.position_ref[0];
                state_sp[1] = Command_Now.position_ref[1];
                state_sp[2] = Command_Now.position_ref[2];
                yaw_sp = Command_Now.yaw_ref;
                send_pos_setpoint(state_sp, yaw_sp);
            }else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            {
                state_sp[0] = Command_Now.velocity_ref[0];
                state_sp[1] = Command_Now.velocity_ref[1];
                state_sp[2] = Command_Now.position_ref[2];
                yaw_sp = Command_Now.yaw_ref;
                send_vel_xy_pos_z_setpoint(state_sp, yaw_sp);

                // 机体系控制,暂缺
                if(false)
                {
                    //xy velocity mode
                    float d_vel_body[2] = {Command_Now.velocity_ref[0], Command_Now.velocity_ref[1]};         //the desired xy velocity in Body Frame
                    float d_vel_enu[2];                   //the desired xy velocity in NED Frame

                    //根据无人机当前偏航角进行坐标系转换
                    rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
                    Command_Now.position_ref[0] = 0;
                    Command_Now.position_ref[1] = 0;
                    Command_Now.position_ref[2] = Command_Now.position_ref[2];
                    Command_Now.velocity_ref[0] = d_vel_enu[0];
                    Command_Now.velocity_ref[1] = d_vel_enu[1];
                    Command_Now.velocity_ref[2] = 0.0;
                    state_sp = Eigen::Vector3d(Command_Now.velocity_ref[0],Command_Now.velocity_ref[1],Command_Now.position_ref[2]);
                    yaw_sp = _DroneState.attitude[2] + Command_Now.yaw_ref;
                }
            }else if(Command_Now.Move_mode == prometheus_msgs::SwarmCommand::TRAJECTORY)
            {
                state_sp << Command_Now.position_ref[0],Command_Now.position_ref[1],Command_Now.position_ref[2];
                // z轴定高飞行，速度为0
                state_sp_extra << Command_Now.velocity_ref[0],Command_Now.velocity_ref[1], 0.0 ;
                yaw_sp = Command_Now.yaw_ref;
                send_pos_vel_xyz_setpoint(state_sp , state_sp_extra, yaw_sp);
            }
            else
            {
                pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_name, "Wrong swarm command!");
            }

            break;

        case prometheus_msgs::SwarmCommand::User_Mode1:

            break;
        }

        Command_Last = Command_Now;
        rate.sleep();
    }
}